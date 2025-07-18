// src/rtabmap_pose_publisher.cpp

// c++ header
#include <algorithm>

// rtabmap header
#include <rtabmap_msgs/msg/info.hpp>

// ros header 
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Aeirobot header
#include "aeirobot_toolbox/qos_profiles.hpp"
#include <alice4_localization_msgs/msg/pose_with_info_stamped.hpp>

using InfoMsg = rtabmap_msgs::msg::Info;
using OdomMsg = nav_msgs::msg::Odometry;

class PosePublisherNode : public rclcpp::Node
{
public:
  PosePublisherNode()
  : Node("rtabmap_pose_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 파라미터로부터 프레임 ID 읽기
    base_frame_id_ = this->declare_parameter<std::string>("correction_base_frame_id", "pelvis_waist");
    odom_frame_id_ = this->declare_parameter<std::string>("correction_odom_frame_id", "odom");
    min_score_     = this->declare_parameter<double>("min_loop_score", 0.4);

    // 파라미터로부터 토픽 이름 읽기
    info_topic     = this->declare_parameter<std::string>("info_topic", "rtabmap/info");
    zed_odom_topic = this->declare_parameter<std::string>("zed_odom_topic", "/zed_odom");

    map_to_odom_topic = this->declare_parameter<std::string>("map_to_odom_topic", "/aeirobot/localization/map_to_odom");
    global_pose_topic = this->declare_parameter<std::string>("global_pose_topic", "/aeirobot/localization/map_to_pelvis_visual_slam");   // 3d map -> pelvis
    global_pose_topic_2d = this->declare_parameter<std::string>("global_pose_topic_2d", "/aeirobot/localization/pose_visual_slam");      // 2d map -> pelvis

    // SUBSCRIBER //
    info_sub_ = this->create_subscription<InfoMsg>(info_topic, 10, std::bind(&PosePublisherNode::InfoCallback, this, std::placeholders::_1));
    zed_odom_sub_ = this->create_subscription<OdomMsg>(zed_odom_topic, aeirobot::qos_sensor_profile, std::bind(&PosePublisherNode::ZedCallback, this, std::placeholders::_1));

    // PUBLISHER //
    correct_m2o_pub_ = this->create_publisher<OdomMsg>(map_to_odom_topic, aeirobot::qos_sensor_profile);          
    correct_m2p_pub_ = this->create_publisher<OdomMsg>(global_pose_topic, aeirobot::qos_sensor_profile);
    correct_m2p_2d_pub_ = this->create_publisher<alice4_localization_msgs::msg::PoseWithInfoStamped>(global_pose_topic_2d, aeirobot::qos_sensor_profile);

    RCLCPP_INFO(this->get_logger(), "rtabmap_pose_publisher node started");
  }

private:
  void InfoCallback(const InfoMsg::SharedPtr msg)
  {
    auto stat = [&](const std::string & key)->double {
      for (size_t i = 0; i < msg->stats_keys.size(); ++i)
        if (msg->stats_keys[i] == key && i < msg->stats_values.size())
          return msg->stats_values[i];
      return 0.0;
    };

    // ✅ 총 연산 시간 로깅  RtabmapROS/TimeTotal/ms   Timing/Total/ms
    last_total_time_ = stat("RtabmapROS/TimeTotal/ms") / 1000.0;
    // RCLCPP_INFO(this->get_logger(), "⏱ RTAB-Map 전체 처리 시간 (Timing/Total): %.4f sec", last_total_time_);

    double vis_inliers  = stat("Loop/Visual_inliers/");
    double inlier_ratio = stat("Loop/Visual_inliers_ratio/");
    double score = 0.5 * std::min(vis_inliers / 30.0, 1.0) + 0.5 * std::clamp(inlier_ratio , 0.0 , 1.0);

    if ((msg->loop_closure_id > 0 || msg->proximity_detection_id > 0) &&
        score >= min_score_)
    {
      // 1. map→odom 저장
      tf2::fromMsg(msg->odom_cache.map_to_odom, corrected_map_to_odom_);
      // has_corrected_map_to_odom_ = true;

      // 2. 메시지 퍼블리시: map→odom
      OdomMsg odom_msg;
      odom_msg.header.stamp    = msg->header.stamp;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id  = odom_frame_id_;

      odom_msg.pose.pose.position.x = corrected_map_to_odom_.getOrigin().x();
      odom_msg.pose.pose.position.y = corrected_map_to_odom_.getOrigin().y();
      odom_msg.pose.pose.position.z = corrected_map_to_odom_.getOrigin().z();
      odom_msg.pose.pose.orientation = tf2::toMsg(corrected_map_to_odom_.getRotation());

      std::fill(std::begin(odom_msg.pose.covariance), std::end(odom_msg.pose.covariance), 0.0);
      correct_m2o_pub_->publish(odom_msg);

      try
      {
        /* 1. 현재 odom→pelvis TF 읽기 */
        geometry_msgs::msg::TransformStamped odom_to_pelvis_msg =
            tf_buffer_.lookupTransform(                     
                odom_frame_id_, base_frame_id_,             
                msg->header.stamp,                        
                rclcpp::Duration::from_seconds(0.05));  
            
        tf2::Transform tf_odom_to_pelvis;
        tf2::fromMsg(odom_to_pelvis_msg.transform, tf_odom_to_pelvis);
            
        /* 2. 보정된 map→odom * odom→pelvis = map→pelvis */
        tf2::Transform tf_map_to_pelvis = corrected_map_to_odom_ * tf_odom_to_pelvis;
            
        tf2::Quaternion q = tf_map_to_pelvis.getRotation();
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
        /* 3. map 기준 pelvis 좌표를 로그로 출력 */
        RCLCPP_INFO(this->get_logger(),
            "\033[1;32m ✔ map→pelvis 보정(score=%.3f) x=%.3f, y=%.3f, z=%.3f, yaw=%.3f deg \033[0m",
            score,
            tf_map_to_pelvis.getOrigin().x(),
            tf_map_to_pelvis.getOrigin().y(),
            tf_map_to_pelvis.getOrigin().z(),
            yaw*180/3.1416);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_WARN(this->get_logger(),
            "map→pelvis 로그용 TF 조회 실패: %s", ex.what());
      }
    }
    else if (score < min_score_) 
    {
      // RCLCPP_INFO(this->get_logger(), "✔ map→odom 보정 퍼블리시 무시, score 낮음 (score=%.3f)", score);

    }
  }


  void ZedCallback(const OdomMsg::SharedPtr msg)
  {
    tf2::Transform tf_odom_to_pelvis;
    tf2::fromMsg(msg->pose.pose, tf_odom_to_pelvis);

    tf2::Transform tf_map_to_pelvis = corrected_map_to_odom_ * tf_odom_to_pelvis;

    // ---------- 기존 3D pose 퍼블리시 (Odometry) ----------
    OdomMsg corrected_pose;
    corrected_pose.header.stamp = msg->header.stamp;
    corrected_pose.header.frame_id = "map";
    corrected_pose.child_frame_id = base_frame_id_;

    corrected_pose.pose.pose.position.x = tf_map_to_pelvis.getOrigin().x();
    corrected_pose.pose.pose.position.y = tf_map_to_pelvis.getOrigin().y();
    corrected_pose.pose.pose.position.z = tf_map_to_pelvis.getOrigin().z();
    corrected_pose.pose.pose.orientation = tf2::toMsg(tf_map_to_pelvis.getRotation());

    std::fill(std::begin(corrected_pose.pose.covariance), std::end(corrected_pose.pose.covariance), 0.0);
    correct_m2p_pub_->publish(corrected_pose);
    // ---------- 기존 3D pose 퍼블리시 (Odometry) ----------

    // ---------- 추가: 2D pose 퍼블리시 (PoseWithInfoStamped) ----------
    alice4_localization_msgs::msg::PoseWithInfoStamped pose_2d_msg;
    pose_2d_msg.header.stamp = msg->header.stamp;
    pose_2d_msg.header.frame_id = "map";

    pose_2d_msg.pose.x = tf_map_to_pelvis.getOrigin().x();
    pose_2d_msg.pose.y = tf_map_to_pelvis.getOrigin().y();

    tf2::Quaternion q = tf_map_to_pelvis.getRotation();
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_2d_msg.pose.theta = yaw;

    // 🔁 최신 delta_time 저장해둔 값 사용 (아래 참고)
    pose_2d_msg.delta_time = last_total_time_;

    // optional: info 필드 초기화
    // pose_2d_msg.info.clear();

    correct_m2p_2d_pub_->publish(pose_2d_msg);
  }

  std::string base_frame_id_, odom_frame_id_;
  std::string info_topic, zed_odom_topic, map_to_odom_topic, global_pose_topic, global_pose_topic_2d; 
  double min_score_;

  rclcpp::Subscription<InfoMsg>::SharedPtr info_sub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr zed_odom_sub_;

  rclcpp::Publisher<OdomMsg>::SharedPtr correct_m2o_pub_;   
  rclcpp::Publisher<OdomMsg>::SharedPtr correct_m2p_pub_;
  rclcpp::Publisher<alice4_localization_msgs::msg::PoseWithInfoStamped>::SharedPtr correct_m2p_2d_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  tf2::Transform corrected_map_to_odom_;
  bool has_corrected_map_to_odom_ = false;

  double roll, pitch, yaw;
  double last_total_time_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
