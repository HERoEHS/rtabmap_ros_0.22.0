#
# To avoid log buffering:
# "stdbuf -o L ros2 launch rtabmap_launch rtabmap.launch.py ..."
#

import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory

#Based on https://answers.ros.org/question/363763/ros2-how-best-to-conditionally-include-a-prefix-in-a-launchpy-file/
class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition == True or self.condition == 'true' or self.condition == 'True':
            return self.text_if
        else:
            return self.text_else
            
class ConditionalBool(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> bool:
        if self.condition:
            return self.text_if
        else:
            return self.text_else
            
def launch_setup(context, *args, **kwargs):      
               
    return [
        DeclareLaunchArgument('depth', default_value=ConditionalText('false', 'true', IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true'"]))._predicate_func(context)), description=''),
        DeclareLaunchArgument('subscribe_rgb', default_value=LaunchConfiguration('depth'), description=''),
        DeclareLaunchArgument('args',  default_value=LaunchConfiguration('rtabmap_args'), description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'),
        DeclareLaunchArgument('sync_queue_size',  default_value=LaunchConfiguration('queue_size'), description='Queue size of topic synchronizers.'),
        DeclareLaunchArgument('qos_image',       default_value=LaunchConfiguration('qos'), description='Specific QoS used for image input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_camera_info', default_value=LaunchConfiguration('qos'), description='Specific QoS used for camera info input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_scan',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for scan input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_odom',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for odometry input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_user_data',   default_value=LaunchConfiguration('qos'), description='Specific QoS used for user input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_imu',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for imu input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_gps',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for gps input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        
        DeclareLaunchArgument('odom_log_level',  default_value=LaunchConfiguration('log_level'), description='Specific ROS logger level for odometry node.'),
        
        #These arguments should not be modified directly, see referred topics without "_relay" suffix above
        DeclareLaunchArgument('rgb_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('rgb_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('rgb_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('depth_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('depth_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('depth_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('left_image_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('left_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('left_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('right_image_topic_relay',      default_value=ConditionalText(''.join([LaunchConfiguration('right_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('right_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('rgbd_topic_relay',      default_value=ConditionalText(''.join(LaunchConfiguration('rgbd_topic').perform(context)), ''.join([LaunchConfiguration('rgbd_topic').perform(context), "_relay"]), LaunchConfiguration('rgbd_sync').perform(context)), description='Should not be modified manually!'),
    
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        # Relays Stereo
        Node(
            package='image_transport', executable='republish', name='republish_left',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
            remappings=[
                (['in/', LaunchConfiguration('rgb_image_transport')], [LaunchConfiguration('left_image_topic'), '/', LaunchConfiguration('rgb_image_transport')]),
                ('out', LaunchConfiguration('left_image_topic_relay'))], 
            arguments=[LaunchConfiguration('rgb_image_transport'), 'raw'],
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='image_transport', executable='republish', name='republish_right',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
            remappings=[
                (['in/', LaunchConfiguration('rgb_image_transport')], [LaunchConfiguration('right_image_topic'), '/', LaunchConfiguration('rgb_image_transport')]),
                ('out', LaunchConfiguration('right_image_topic_relay'))], 
            arguments=[LaunchConfiguration('rgb_image_transport'), 'raw'],
            namespace=LaunchConfiguration('namespace')),
        Node(
            package='rtabmap_sync', executable='stereo_sync', name="stereo_sync", output="screen",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true' and '", LaunchConfiguration('rgbd_sync'), "' == 'true'"])),
            parameters=[{
                "approx_sync": LaunchConfiguration('approx_rgbd_sync'),
                "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos": LaunchConfiguration('qos_image'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info')}],
            remappings=[
                ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
                ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
                ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
                ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
                ("rgbd_image", LaunchConfiguration('rgbd_topic_relay'))],
            namespace=LaunchConfiguration('namespace')),
            
             
        # Stereo odometry
        # Node(
        #     package='rtabmap_odom', executable='stereo_odometry', name="stereo_odometry", output="screen",
        #     condition=IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' != 'true' and '", LaunchConfiguration('visual_odometry'), "' == 'true' and '", LaunchConfiguration('stereo'), "' == 'true'"])),
        #     parameters=[{
        #         #2D odom으로 제한 
        #         "Odom/2D": True ,
        #         "frame_id": LaunchConfiguration('frame_id'),
        #         "odom_frame_id": LaunchConfiguration('vo_frame_id'),
        #         "publish_tf": LaunchConfiguration('publish_tf_odom'),
        #         "wait_for_transform": LaunchConfiguration('wait_for_transform'),
        #         "wait_imu_to_init": LaunchConfiguration('wait_imu_to_init'),
        #         "approx_sync": LaunchConfiguration('approx_sync'),
        #         "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
        #         "config_path": LaunchConfiguration('cfg').perform(context),
        #         "topic_queue_size": LaunchConfiguration('topic_queue_size'),
        #         "sync_queue_size": LaunchConfiguration('sync_queue_size'),
        #         "qos": LaunchConfiguration('qos_image'),
        #         "qos_camera_info": LaunchConfiguration('qos_camera_info'),
        #         "qos_imu": LaunchConfiguration('qos_imu'),    
        #         }],
        #     remappings=[
        #         ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
        #         ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
        #         ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
        #         ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
        #         ("odom", LaunchConfiguration('odom_topic')),
        #         ("imu", LaunchConfiguration('imu_topic'))],
        #     arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.stereo_odometry:=', LaunchConfiguration('odom_log_level')], "--log-level", ['stereo_odometry:=', LaunchConfiguration('odom_log_level')]],
        #     prefix=LaunchConfiguration('launch_prefix'),
        #     namespace=LaunchConfiguration('namespace')),
            

        ### Visual SLAM ###
        Node(
            package='rtabmap_slam', executable='rtabmap', name="rtabmap", output="screen",
            parameters=[{
                "subscribe_stereo": LaunchConfiguration('stereo'),
                "subscribe_odom_info": ConditionalBool(True, False, IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' == 'true' or '", LaunchConfiguration('visual_odometry'), "' == 'true'"]))._predicate_func(context)).perform(context),
                "frame_id": LaunchConfiguration('frame_id'),
                "map_frame_id": LaunchConfiguration('map_frame_id'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id').perform(context),
                "publish_tf": LaunchConfiguration('publish_tf_map'),
                "initial_pose": LaunchConfiguration('initial_pose'),
                "database_path": LaunchConfiguration('database_path'),
                "ground_truth_frame_id": LaunchConfiguration('ground_truth_frame_id').perform(context),
                "ground_truth_base_frame_id": LaunchConfiguration('ground_truth_base_frame_id').perform(context),
                "odom_tf_angular_variance": LaunchConfiguration('odom_tf_angular_variance'),
                "odom_tf_linear_variance": LaunchConfiguration('odom_tf_linear_variance'),
                "odom_sensor_sync": LaunchConfiguration('odom_sensor_sync'),
                "wait_for_transform": LaunchConfiguration('wait_for_transform'),
                "approx_sync": LaunchConfiguration('approx_sync'),
                "config_path": LaunchConfiguration('cfg').perform(context),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos_image": LaunchConfiguration('qos_image'),
                "qos_odom": LaunchConfiguration('qos_odom'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info'),
                "qos_imu": LaunchConfiguration('qos_imu'),
                "landmark_linear_variance": LaunchConfiguration('tag_linear_variance'),
                "landmark_angular_variance": LaunchConfiguration('tag_angular_variance'),
                "Mem/IncrementalMemory": ConditionalText("true", "false", IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' != 'true'"]))._predicate_func(context)).perform(context),
                "Mem/InitWMWithAllNodes": ConditionalText("true", "false", IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'true'"]))._predicate_func(context)).perform(context),
                # "Grid/MaxObstacleHeight": "0.9",
                'Grid/3D': "false",  # 2D Occupancy Grid 활성화
                "Reg/Force3DoF": "true",
                'Reg/Strategy':'0',         # "0=Vis, 1=Icp, 2=VisIcp"
                'Kp/DetectorStrategy':'11',  # "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector"
                'Vis/FeatureType':'11',      # "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector"
                'Vis/MaxFeatures': '500',     # 기본값 1000
                'ORB/Gpu': 'true',
                'Kp/MaxDepth': '15.0',
                "Mem/ImagePostDecimation": "2",   # 맵 저장할 때 이미지 다운스케일해서 저장함. 1: 원본, 2 3 4... 1/2 1/3 1/4로 다운스케일 하겠다
                "Mem/ImagePreDecimation": "2",    # 실시간에서 이미지를 다운스케일해서 사용함. 1: 원본, 2 3 4... 1/2 1/3 1/4로 다운스케일 하겠다  
                'SuperPoint/ModelPath': '/home/orin/vslam_ws/src/superpoint_v1.pt',
                'PyMatcher/Path': '/home/orin/vslam_ws/src/SuperGluePretrainedNetwork/rtabmap_superglue.py',
                'Vis/CorGuessWinSize': '0',   # 기본값 40
                'Vis/CorNNType': '6',   #  기본값은 1, kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7
                'Reg/RepeatOnce': 'false',  # 기본값 true
                'PyMatcher/Model': 'outdoor',      # indoor outdoor
                'Vis/DepthMaskFloorThr': '0.4',   # 뎁스값 추정해서 바닥은 필터링해서 없애는 거 ...아직 잘 모르겠음ㅠ
                'Kp/MaxFeatures': '500',
                'Mem/STMSize': '20',               # 10
                'Mem/RehearsalSimilarity': '0.8',  # 0.6
            }],
            remappings=[
                ("map", LaunchConfiguration('map_topic')),
                ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
                ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
                ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
                ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic')),
                ],
            arguments=[LaunchConfiguration("args"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.rtabmap:=', LaunchConfiguration('log_level')], "--log-level", ['rtabmap:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        ### Rtabmap GUI ###
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', name="rtabmap_viz", output='screen',
            parameters=[{
                "subscribe_stereo": LaunchConfiguration('stereo'),
                "subscribe_odom_info": ConditionalBool(True, False, IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' == 'true' or '", LaunchConfiguration('visual_odometry'), "' == 'true'"]))._predicate_func(context)).perform(context),
                "frame_id": LaunchConfiguration('frame_id'),
                "odom_frame_id": LaunchConfiguration('odom_frame_id').perform(context),
                "wait_for_transform": LaunchConfiguration('wait_for_transform'),
                "approx_sync": LaunchConfiguration('approx_sync'),
                "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                "qos_image": LaunchConfiguration('qos_image'),
                "qos_odom": LaunchConfiguration('qos_odom'),
                "qos_camera_info": LaunchConfiguration('qos_camera_info'),
            }],
            remappings=[
                ("left/image_rect", LaunchConfiguration('left_image_topic_relay')),
                ("right/image_rect", LaunchConfiguration('right_image_topic_relay')),
                ("left/camera_info", LaunchConfiguration('left_camera_info_topic')),
                ("right/camera_info", LaunchConfiguration('right_camera_info_topic')),
                ("odom", LaunchConfiguration('odom_topic'))],
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            arguments=[LaunchConfiguration("gui_cfg"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.rtabmap_viz:=', LaunchConfiguration('log_level')], "--log-level", ['rtabmap_viz:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        ### Rviz ###
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
        
        ### Rtabmap Util ###
        # Node(
        #     package='rtabmap_util', executable='point_cloud_xyzrgb', name="point_cloud_xyzrgb", output='screen',
        #     condition=IfCondition(LaunchConfiguration("rviz")),
        #     parameters=[{
        #         "decimation": 1,
        #         "voxel_size": 0.05,
        #         "approx_sync": LaunchConfiguration('approx_sync'),
        #         "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval')
        #     }],
        #     remappings=[
        #         ('left/image', LaunchConfiguration('left_image_topic_relay')),
        #         ('right/image', LaunchConfiguration('right_image_topic_relay')),
        #         ('left/camera_info', LaunchConfiguration('left_camera_info_topic')),
        #         ('right/camera_info', LaunchConfiguration('right_camera_info_topic')),
        #         ('rgb/image', LaunchConfiguration('rgb_topic_relay')),
        #         ('depth/image', LaunchConfiguration('depth_topic_relay')),
        #         ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        #         ('rgbd_image', LaunchConfiguration('rgbd_topic_relay')),
        #         ('cloud', 'voxel_cloud')]),
        ]

def generate_launch_description():
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )
    
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument('stereo', default_value='true', description='Use stereo input instead of RGB-D.'),

        DeclareLaunchArgument('localization', default_value='true', description='Launch in localization mode.'),
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),

        # odom tf 보정 할지말지 변수
        DeclareLaunchArgument('odom_correction', default_value='false', description='loop closing 상황에서 odom tf 옮길건지 말건지 선택하는 변수'),
        
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('log_level',    default_value='info', description="ROS logging level (debug, info, warn, error). For RTAB-Map\'s logger level, use \"args\" argument."),

        # Config files  ~/.rtabmap/test.ini
        DeclareLaunchArgument('cfg',      default_value='',                        description='To change RTAB-Map\'s parameters, set the path of config file (*.ini) generated by the standalone app.'),
        DeclareLaunchArgument('gui_cfg',  default_value='~/.ros/rtabmap_gui.ini',  description='Configuration path of rtabmap_viz.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,               description='Configuration path of rviz2.'),

        DeclareLaunchArgument('frame_id',       default_value='zed_camera_link',          description='Fixed frame id of the robot (base frame), you may set "base_link" or "base_footprint" if they are published. For camera-only config, this could be "camera_link".'),
        DeclareLaunchArgument('odom_frame_id',  default_value='odom',                   description='If set, TF is used to get odometry instead of the topic.'),
        DeclareLaunchArgument('map_frame_id',   default_value='map',                description='Output map frame id (TF).'),
        DeclareLaunchArgument('map_topic',      default_value='map',                description='Map topic name.'),
        DeclareLaunchArgument('publish_tf_map', default_value='true',               description='Publish TF between map and odomerty.'),
        DeclareLaunchArgument('namespace',      default_value='rtabmap',            description=''),
        DeclareLaunchArgument('database_path',  default_value='~/.ros/mapping/rtabmap.db',  description='Where is the map saved/loaded.'),
        DeclareLaunchArgument('topic_queue_size', default_value='10',                description='Queue size of individual topic subscribers.'),
        DeclareLaunchArgument('queue_size',     default_value='10',                 description='Backward compatibility, use "sync_queue_size" instead.'),
        DeclareLaunchArgument('qos',            default_value='2',                  description='General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('wait_for_transform', default_value='0.2',            description=''),
        DeclareLaunchArgument('rtabmap_args',   default_value='',                   description='Backward compatibility, use "args" instead.'),
        DeclareLaunchArgument('launch_prefix',  default_value='',                   description='For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'),
        DeclareLaunchArgument('output',         default_value='screen',             description='Control node output (screen or log).'),
        DeclareLaunchArgument('initial_pose',   default_value='',                   description='Set an initial pose (only in localization mode). Format: "x y z roll pitch yaw" or "x y z qx qy qz qw". Default: see "RGBD/StartAtOrigin" doc'),

        ### 이게 있어야 맵이 받아지는듯?? ###
        DeclareLaunchArgument('ground_truth_frame_id',      default_value='map', description='e.g., "world"'),
        DeclareLaunchArgument('ground_truth_base_frame_id', default_value='tracker', description='e.g., "tracker", a fake frame matching the frame "frame_id" (but on different TF tree)'),
        
        DeclareLaunchArgument('approx_sync',  default_value='true',            description='If timestamps of the input topics should be synchronized using approximate or exact time policy.'),
        DeclareLaunchArgument('approx_sync_max_interval',  default_value='0.0', description='(sec) 0 means infinite interval duration (used with approx_sync=true)'),

        # RGB-D related topics
        DeclareLaunchArgument('rgb_topic',           default_value='/camera/rgb/image_rect_color',       description=''),
        DeclareLaunchArgument('depth_topic',         default_value='/camera/depth_registered/image_raw', description=''),
        DeclareLaunchArgument('camera_info_topic',   default_value='/camera/rgb/camera_info',            description=''),
        
        # Stereo related topics
        DeclareLaunchArgument('stereo_namespace',        default_value='', description=''),
        # DeclareLaunchArgument('left_image_topic',        default_value=[LaunchConfiguration('stereo_namespace'), '/left_image_topic'], description=''),
        # DeclareLaunchArgument('right_image_topic',       default_value=[LaunchConfiguration('stereo_namespace'), '/right_image_topic'], description=''),
        # DeclareLaunchArgument('left_camera_info_topic',  default_value=[LaunchConfiguration('stereo_namespace'), '/left_camera_info'], description=''),
        # DeclareLaunchArgument('right_camera_info_topic', default_value=[LaunchConfiguration('stereo_namespace'), '/right_camera_info'], description=''),
        
        DeclareLaunchArgument('left_image_topic',        default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_left_image'], description=''),
        DeclareLaunchArgument('right_image_topic',       default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_right_image'], description=''),
        DeclareLaunchArgument('left_camera_info_topic',  default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_left_camera_info'], description=''),
        DeclareLaunchArgument('right_camera_info_topic', default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_right_camera_info'], description=''),
        
                
        # Use Pre-sync RGBDImage format
        DeclareLaunchArgument('rgbd_sync',        default_value='false',      description='Pre-sync rgb_topic, depth_topic, camera_info_topic.'),
        DeclareLaunchArgument('approx_rgbd_sync', default_value='false',       description='false=exact synchronization.'),
        DeclareLaunchArgument('subscribe_rgbd',   default_value=LaunchConfiguration('rgbd_sync'), description='Already synchronized RGB-D related topic, e.g., with rtabmap_sync/rgbd_sync nodelet.'),
        DeclareLaunchArgument('rgbd_topic',       default_value='rgbd_image', description=''),
        DeclareLaunchArgument('depth_scale',      default_value='1.0',        description=''),
        
        # Image topic compression
        DeclareLaunchArgument('compressed',            default_value='false', description='If you want to subscribe to compressed image topics'),
        DeclareLaunchArgument('rgb_image_transport',   default_value='compressed', description='Common types: compressed, theora (see "rosrun image_transport list_transports")'),
        DeclareLaunchArgument('depth_image_transport', default_value='compressedDepth', description='Depth compatible types: compressedDepth (see "rosrun image_transport list_transports")'),
       
            
        # Odometry
        DeclareLaunchArgument('visual_odometry',            default_value='false',  description='Launch rtabmap visual odometry node.'),
        DeclareLaunchArgument('icp_odometry',               default_value='false', description='Launch rtabmap icp odometry node.'),
        DeclareLaunchArgument('odom_topic',                 default_value='/zed_odom',  description='Odometry topic name.'),
        DeclareLaunchArgument('vo_frame_id',                default_value=LaunchConfiguration('odom_topic'), description='Visual/Icp odometry frame ID for TF.'),
        DeclareLaunchArgument('publish_tf_odom',            default_value='false',  description=''),
        DeclareLaunchArgument('odom_tf_angular_variance',   default_value='0.01',    description='If TF is used to get odometry, this is the default angular variance'),
        DeclareLaunchArgument('odom_tf_linear_variance',    default_value='0.001',   description='If TF is used to get odometry, this is the default linear variance'),
        DeclareLaunchArgument('odom_args',                  default_value='',      description='More arguments for odometry (overwrite same parameters in rtabmap_args).'),
        DeclareLaunchArgument('odom_sensor_sync',           default_value='false', description=''),
        DeclareLaunchArgument('odom_guess_frame_id',        default_value='',      description=''),
        DeclareLaunchArgument('odom_guess_min_translation', default_value='0.0',   description=''),
        DeclareLaunchArgument('odom_guess_min_rotation',    default_value='0.0',   description=''),
        
        # imu
        DeclareLaunchArgument('imu_topic',        default_value='/zed_imu', description='Used with VIO approaches and for SLAM graph optimization (gravity constraints).'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false',     description=''),
        
        
        # Tag/Landmark
        DeclareLaunchArgument('tag_topic',            default_value='/detections', description='AprilTag topic async subscription. This is used for SLAM graph optimization and loop closure detection. Landmark poses are also published accordingly to current optimized map. Required: Remove optional frame name parameters from apriltag\'s cfg file so that TF frame can be deducted from topic\'s family and id.'),
        DeclareLaunchArgument('tag_linear_variance',  default_value='0.0001',          description=''),
        DeclareLaunchArgument('tag_angular_variance', default_value='9999.0',            description='>=9999 means rotation is ignored in optimization, when rotation estimation of the tag is not reliable or not computed.'),
        DeclareLaunchArgument('fiducial_topic',       default_value='/fiducial_transforms', description='aruco_detect async subscription, use tag_linear_variance and tag_angular_variance to set covariance.'),
        OpaqueFunction(function=launch_setup)
    ])