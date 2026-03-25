#
# RTABMAP + SLAM Manager GUI 런치파일
#
# 기존 rtabmap_local_test.launch.py와의 차이점:
#   1. localization/extend_map argument 제거 → 항상 매핑 모드로 시작
#   2. 기본 localization 모드: Mem/IncrementalMemory: "false", Mem/InitWMWithAllNodes: "true"
#   3. GUI (SLAM Manager)가 런타임에 서비스 호출로 모드 전환
#   4. database_path 기본값을 ~/.ros/maps/ 디렉토리로 설정
#
# 사용법:
#   ros2 launch rtabmap_launch rtabmap_with_gui.launch.py
#   → RTABMAP이 매핑 모드로 시작되고, SLAM Manager GUI가 자동 실행됨
#   → GUI에서 매핑/위치추정 모드 전환, 맵 저장/불러오기, 그리드맵 추출 가능
#

import os

# ROS_WS 환경변수 기반 경로 설정
ROS_WS = os.environ.get('ROS_WS', os.path.expanduser('~/revision_daim_ws'))
SLAM_MANAGER_CONFIG = os.path.join(ROS_WS, 'src', 'alice_navigation', 'localization', 'rtabmap_slam_manager', 'config')
FEATURE_EXTRACTORS = os.path.join(ROS_WS, 'src', 'alice_navigation', 'localization', 'feature_extractors')

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from typing import Text
from ament_index_python.packages import get_package_share_directory


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

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),


        ### SLAM Manager GUI (먼저 실행 → RTABMAP 연결 시 즉시 pause) ###
        Node(
            package='rtabmap_slam_manager',
            executable='slam_manager_web',
            name='slam_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration("slam_manager_gui")),
        ),

        ### Visual SLAM (3초 지연 → GUI가 먼저 준비되어 시작 즉시 pause 가능) ###
        TimerAction(period=3.0, actions=[
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

                # ============================================================
                # GUI 모드 고정 파라미터
                # 매핑 모드로 시작 → GUI에서 set_mode_localization/set_mode_mapping 서비스로 전환
                # ============================================================
                "Mem/IncrementalMemory": "false",      # 기본 localization 모드 (GUI에서 매핑 모드로 전환 가능)
                "Mem/InitWMWithAllNodes": "true",      # 기존 맵 전체를 WM에 로드

                'Mem/RecentWmRatio': '0.3',
                "odom_correction": LaunchConfiguration('odom_correction'),

                # Grid Map 높이 필터링 파라미터 (바닥면 제거용)
                'Grid/MinGroundHeight': '0.0',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '1.0',
                'Grid/NormalsSegmentation': 'false',
                'Grid/RayTracing': 'true',
                # 'Grid/CellSize': '0.05',
                # 'Grid/RangeMax': '5.0',
                # 'Grid/RangeMin': '0.3',

                'Grid/3D': "false",
                "Reg/Force3DoF": "true",
                'RGBD/ForceOdom3DoF': 'true',
                'Reg/Strategy':'0',
                'Kp/DetectorStrategy':'11',
                'Vis/FeatureType':'11',
                'Vis/MaxFeatures': '500',
                'ORB/Gpu': 'false',
                "Mem/ImagePostDecimation": "2",
                "Mem/ImagePreDecimation": "2",
                'SuperPoint/ModelPath': os.path.join(FEATURE_EXTRACTORS, 'superpoint_v1.pt'),
                'PyMatcher/Path': os.path.join(FEATURE_EXTRACTORS, 'SuperGluePretrainedNetwork', 'rtabmap_superglue.py'),
                'Vis/CorGuessWinSize': '0',
                'Vis/CorNNType': '6',
                'Reg/RepeatOnce': 'false',
                # 'Vis/DepthMaskFloorThr': '0.4',
                'Kp/MaxFeatures': '500',
                'Mem/RehearsalSimilarity': '0.6',
                'RGBD/OptimizeMaxError': '0.0',
                'Optimizer/Robust': 'true',
                'Rtabmap/DetectionRate': '1',
                'Kp/MaxDepth': '8.0',

                ## 레전드 파라미터 ##
                'SuperPoint/Threshold': '0.005',
                'SuperPoint/NMSRadius': '2',
                'PyMatcher/Iterations': '40',
                'PyMatcher/Threshold': '0.15',
                'PyMatcher/Model': 'indoor',
            }],
            remappings=[
                ("map", LaunchConfiguration('map_topic')),
                ("left/image_rect", "/aeirobot/vslam_left_image"),
                ("right/image_rect", "/aeirobot/vslam_right_image"),
                ("left/camera_info", "/aeirobot/vslam_left_camera_info"),
                ("right/camera_info", "/aeirobot/vslam_right_camera_info"),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic')),
                ],
            arguments=[LaunchConfiguration("args"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.rtabmap:=', LaunchConfiguration('log_level')], "--log-level", ['rtabmap:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),
        ]),  # TimerAction 닫기

        ### Rtabmap GUI (원본 rtabmap_viz, 기본 비활성) ###
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
                ("left/image_rect", "/aeirobot/vslam_left_image"),
                ("right/image_rect", "/aeirobot/vslam_right_image"),
                ("left/camera_info", "/aeirobot/vslam_left_camera_info"),
                ("right/camera_info", "/aeirobot/vslam_right_camera_info"),
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
        ]

def generate_launch_description():

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )

    return LaunchDescription([

        # Arguments
        DeclareLaunchArgument('stereo', default_value='true', description='Use stereo input instead of RGB-D.'),

        ## GUI ON / OFF
        DeclareLaunchArgument('rtabmap_viz',       default_value='false', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',              default_value='false', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('slam_manager_gui',  default_value='true',  description='Launch SLAM Manager GUI.'),

        ## odom tf 보정
        DeclareLaunchArgument('odom_correction', default_value='true', description='loop closing 상황에서 odom tf 옮길건지 말건지 선택하는 변수'),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('log_level',    default_value='info', description="ROS logging level (debug, info, warn, error)."),

        # Config files
        DeclareLaunchArgument('cfg',      default_value='',                        description='RTAB-Map config file path (*.ini).'),
        DeclareLaunchArgument('gui_cfg',  default_value='~/.ros/rtabmap_gui.ini',  description='Configuration path of rtabmap_viz.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,               description='Configuration path of rviz2.'),

        DeclareLaunchArgument('frame_id',       default_value='base_footprint',     description=''),
        DeclareLaunchArgument('odom_frame_id',  default_value='odom',               description=''),
        DeclareLaunchArgument('map_frame_id',   default_value='map',                description=''),
        DeclareLaunchArgument('map_topic',      default_value='map',                description=''),
        DeclareLaunchArgument('publish_tf_map', default_value='true',               description=''),
        DeclareLaunchArgument('namespace',      default_value='rtabmap',            description=''),
        DeclareLaunchArgument('database_path',  default_value='/tmp/rtabmap_runtime.db',  description='런타임 DB (임시, 재부팅 시 삭제됨). 영구 저장은 GUI에서 맵 저장 사용'),
        DeclareLaunchArgument('topic_queue_size', default_value='1',                description=''),
        DeclareLaunchArgument('queue_size',     default_value='10',                 description=''),
        DeclareLaunchArgument('qos',            default_value='2',                  description=''),
        DeclareLaunchArgument('wait_for_transform', default_value='0.2',            description=''),
        DeclareLaunchArgument('rtabmap_args',   default_value='',                   description=''),
        DeclareLaunchArgument('launch_prefix',  default_value='',                   description=''),
        DeclareLaunchArgument('output',         default_value='screen',             description=''),
        DeclareLaunchArgument('initial_pose',   default_value='',                   description=''),

        DeclareLaunchArgument('ground_truth_frame_id',      default_value='', description=''),
        DeclareLaunchArgument('ground_truth_base_frame_id', default_value='', description=''),

        DeclareLaunchArgument('approx_sync',  default_value='true',  description=''),
        DeclareLaunchArgument('approx_sync_max_interval',  default_value='1.0', description=''),

        # Stereo related topics
        DeclareLaunchArgument('stereo_namespace',        default_value='', description=''),
        DeclareLaunchArgument('left_image_topic',        default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_left_image'], description=''),
        DeclareLaunchArgument('right_image_topic',       default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_right_image'], description=''),
        DeclareLaunchArgument('left_camera_info_topic',  default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_left_camera_info'], description=''),
        DeclareLaunchArgument('right_camera_info_topic', default_value=[LaunchConfiguration('stereo_namespace'), '/aeirobot/vslam_right_camera_info'], description=''),

        # Pre-sync RGBDImage format
        DeclareLaunchArgument('rgbd_sync',        default_value='false',      description=''),
        DeclareLaunchArgument('approx_rgbd_sync', default_value='false',      description=''),
        DeclareLaunchArgument('subscribe_rgbd',   default_value=LaunchConfiguration('rgbd_sync'), description=''),
        DeclareLaunchArgument('rgbd_topic',       default_value='rgbd_image', description=''),
        DeclareLaunchArgument('depth_scale',      default_value='1.0',        description=''),

        # Image topic compression
        DeclareLaunchArgument('compressed',            default_value='false', description=''),
        DeclareLaunchArgument('rgb_image_transport',   default_value='compressed', description=''),
        DeclareLaunchArgument('depth_image_transport', default_value='compressedDepth', description=''),

        # Odometry
        DeclareLaunchArgument('visual_odometry',            default_value='false',  description=''),
        DeclareLaunchArgument('icp_odometry',               default_value='false',  description=''),
        DeclareLaunchArgument('odom_topic',                 default_value='/odometry/filtered',  description=''),
        DeclareLaunchArgument('vo_frame_id',                default_value=LaunchConfiguration('odom_topic'), description=''),
        DeclareLaunchArgument('publish_tf_odom',            default_value='false',  description=''),
        DeclareLaunchArgument('odom_tf_angular_variance',   default_value='0.01',   description=''),
        DeclareLaunchArgument('odom_tf_linear_variance',    default_value='0.001',  description=''),
        DeclareLaunchArgument('odom_args',                  default_value='',       description=''),
        DeclareLaunchArgument('odom_sensor_sync',           default_value='false',  description=''),
        DeclareLaunchArgument('odom_guess_frame_id',        default_value='',       description=''),
        DeclareLaunchArgument('odom_guess_min_translation', default_value='0.0',    description=''),
        DeclareLaunchArgument('odom_guess_min_rotation',    default_value='0.0',    description=''),

        # imu
        DeclareLaunchArgument('imu_topic',        default_value='/imu/data_raw', description=''),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false',         description=''),

        OpaqueFunction(function=launch_setup)
    ])
