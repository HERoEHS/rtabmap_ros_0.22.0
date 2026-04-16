#
# RTABMAP 멀티카메라 VSLAM 런치파일
#
# ZED 2i (스테레오, 전면) + Orbbec 336L (RGB-D, 후면) 동시 사용
#
# 구조:
#   [ZED 2i RGB-D]  → rgbd_sync → rgbd_image0 ─┐
#                                                ├─→ rtabmap (rgbd_cameras=2)
#   [Orbbec 336L]   → rgbd_sync → rgbd_image1 ─┘
#
# 빌드 요구사항:
#   rtabmap_ros를 -DRTABMAP_SYNC_MULTI_RGBD=ON 옵션으로 빌드해야 함
#   colcon build --packages-select rtabmap_sync rtabmap_slam rtabmap_odom \
#     --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DCMAKE_BUILD_TYPE=Release
#
# 사용법:
#   ros2 launch rtabmap_launch rtabmap_multicam.launch.py
#

import os

# ROS_WS 환경변수 필수
ROS_WS = os.environ.get('ROS_WS')
if not ROS_WS:
    raise RuntimeError("ROS_WS 환경변수가 설정되지 않았습니다. 예: export ROS_WS=/path/to/ws")
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
        DeclareLaunchArgument('args',  default_value=LaunchConfiguration('rtabmap_args'), description='RTAB-Map parameters or flags like --udebug and --delete_db_on_start/-d'),
        DeclareLaunchArgument('sync_queue_size',  default_value=LaunchConfiguration('queue_size'), description='Queue size of topic synchronizers.'),
        DeclareLaunchArgument('qos_image',       default_value=LaunchConfiguration('qos'), description='Specific QoS used for image input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_camera_info', default_value=LaunchConfiguration('qos'), description='Specific QoS used for camera info input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_scan',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for scan input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_odom',        default_value=LaunchConfiguration('qos'), description='Specific QoS used for odometry input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_user_data',   default_value=LaunchConfiguration('qos'), description='Specific QoS used for user input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_imu',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for imu input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_gps',         default_value=LaunchConfiguration('qos'), description='Specific QoS used for gps input data: 0=system default, 1=Reliable, 2=Best Effort.'),

        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        ### SLAM Manager GUI ###
        Node(
            package='rtabmap_slam_manager',
            executable='slam_manager_web',
            name='slam_manager',
            output='screen',
            condition=IfCondition(LaunchConfiguration("slam_manager_gui")),
        ),

        ### ZED 2i 스테레오 → RGBDImage 변환 ###
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_zed',
            output='screen',
            parameters=[{
                "approx_sync": True,
                "topic_queue_size": 10,
                "sync_queue_size": 10,
                "qos": LaunchConfiguration('qos'),
                "qos_camera_info": LaunchConfiguration('qos'),
            }],
            remappings=[
                ("rgb/image",       LaunchConfiguration('zed_left_image_topic')),
                ("depth/image",     LaunchConfiguration('zed_depth_image_topic')),
                ("rgb/camera_info", LaunchConfiguration('zed_left_camera_info_topic')),
                ("rgbd_image",      "rgbd_image"),
            ],
            namespace='camera_zed',
        ),

        ### Orbbec 336L RGB-D → RGBDImage 변환 ###
        # Orbbec는 BEST_EFFORT(sensor_data) QoS로 퍼블리시 → 구독자도 BEST_EFFORT(2)
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync_orbbec',
            output='screen',
            parameters=[{
                "approx_sync": True,
                "topic_queue_size": 10,
                "sync_queue_size": 10,
                "qos": 2,
                "qos_camera_info": 2,
            }],
            remappings=[
                ("rgb/image",       LaunchConfiguration('orbbec_color_image_topic')),
                ("depth/image",     LaunchConfiguration('orbbec_depth_image_topic')),
                ("rgb/camera_info", LaunchConfiguration('orbbec_camera_info_topic')),
                ("rgbd_image",      "rgbd_image"),
            ],
            namespace='camera_orbbec',
        ),

        ### Visual SLAM (3초 지연 → GUI가 먼저 준비되어 시작 즉시 pause 가능) ###
        TimerAction(period=3.0, actions=[
        Node(
            package='rtabmap_slam', executable='rtabmap', name="rtabmap", output="screen",
            parameters=[{
                # 멀티카메라 RGB-D 모드
                "subscribe_stereo": False,
                "subscribe_rgbd": True,
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "rgbd_cameras": 2,

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

                "Mem/IncrementalMemory":
                    ConditionalText(
                        "true", "false",
                        IfCondition(
                            PythonExpression([
                                "'", LaunchConfiguration('extend_map'), "' == 'true' or ",
                                "'", LaunchConfiguration('localization'), "' != 'true'"
                            ])
                        )._predicate_func(context)
                    ).perform(context),

                "Mem/InitWMWithAllNodes":
                    ConditionalText(
                        "true", "false",
                        IfCondition(
                            PythonExpression([
                                "'", LaunchConfiguration('extend_map'), "' == 'true' or ",
                                "'", LaunchConfiguration('localization'), "' == 'true'"
                            ])
                        )._predicate_func(context)
                    ).perform(context),
                    

                'Mem/RecentWmRatio': '0.3',
                "odom_correction": LaunchConfiguration('odom_correction'),

                # Grid Map 높이 필터링 파라미터 (바닥면 제거용)
                'Grid/MinGroundHeight': '0.0',
                'Grid/MaxGroundHeight': '0.35',
                'Grid/MaxObstacleHeight': '1.0',
                'Grid/NormalsSegmentation': 'false',
                'Grid/RayTracing': 'true',

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
                'Vis/EstimationType': '0',  # 3D-3D registration (OpenGV 없이 멀티카메라 사용)
                'Reg/RepeatOnce': 'false',
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
                ("rgbd_image0", "/camera_zed/rgbd_image"),
                ("rgbd_image1", "/camera_orbbec/rgbd_image"),
                ("odom", LaunchConfiguration('odom_topic')),
                ("imu", LaunchConfiguration('imu_topic')),
                ],
            arguments=[LaunchConfiguration("args"), "--ros-args", "--log-level", [LaunchConfiguration('namespace'), '.rtabmap:=', LaunchConfiguration('log_level')], "--log-level", ['rtabmap:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),
        ]),  # TimerAction 닫기

        ### Rtabmap GUI ###
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', name="rtabmap_viz", output='screen',
            parameters=[{
                "subscribe_odom_info": ConditionalBool(True, False, IfCondition(PythonExpression(["'", LaunchConfiguration('icp_odometry'), "' == 'true' or '", LaunchConfiguration('visual_odometry'), "' == 'true'"]))._predicate_func(context)).perform(context),
                "subscribe_rgbd": True,
                "subscribe_stereo": False,
                "rgbd_cameras": 2,
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
                ("rgbd_image0", "/camera_zed/rgbd_image"),
                ("rgbd_image1", "/camera_orbbec/rgbd_image"),
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
        
        ## Mapping, Localization, Extend mapping 모드 선택
        ### MAPPING MODE:           localization: false, extend_map: false  ###
        ### EXTEND MAPPING MODE:    localization: false, extend_map: true   ###
        ### LOCALIZATION MODE:      localization: true,  extend_map: false  ###
        DeclareLaunchArgument('localization', default_value='false', description='true면 Localization 모드, false면 mapping모드'),
        DeclareLaunchArgument('extend_map', default_value='false', description='LTM 데이터를 모두 WM로 불러온 상태로 추가 맵핑 진행'),
        
        # GUI ON / OFF
        DeclareLaunchArgument('rtabmap_viz',       default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',              default_value='true', description='Launch RVIZ (optional).'),
        DeclareLaunchArgument('slam_manager_gui',  default_value='false',  description='Launch SLAM Manager GUI.'),

        ## odom tf 보정
        DeclareLaunchArgument('odom_correction', default_value='true', description='loop closing 상황에서 odom tf 옮길건지 말건지 선택하는 변수'),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('log_level',    default_value='info', description="ROS logging level (debug, info, warn, error)."),

        # Config files
        DeclareLaunchArgument('cfg',      default_value='',                        description='RTAB-Map config file path (*.ini).'),
        DeclareLaunchArgument('gui_cfg',  default_value='~/.ros/rtabmap_gui.ini',  description='Configuration path of rtabmap_viz.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,               description='Configuration path of rviz2.'),

        DeclareLaunchArgument('frame_id',       default_value='zed_camera_link',     description=''),
        DeclareLaunchArgument('odom_frame_id',  default_value='odom',               description=''),
        DeclareLaunchArgument('map_frame_id',   default_value='map',                description=''),
        DeclareLaunchArgument('map_topic',      default_value='map',                description=''),
        DeclareLaunchArgument('publish_tf_map', default_value='true',               description=''),
        DeclareLaunchArgument('namespace',      default_value='rtabmap',            description=''),
        DeclareLaunchArgument('database_path',  default_value='~/.ros/mapping/multicam_v1.db',  description='런타임 DB (임시, 재부팅 시 삭제됨). 영구 저장은 GUI에서 맵 저장 사용'),
        DeclareLaunchArgument('topic_queue_size', default_value='10',               description=''),
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
        DeclareLaunchArgument('approx_sync_max_interval',  default_value='0.2', description=''),

        # ZED 2i RGB-D 토픽
        DeclareLaunchArgument('zed_left_image_topic',        default_value='/aeirobot/vslam_left_image',       description='ZED 2i 컬러(mono8) 이미지 토픽'),
        DeclareLaunchArgument('zed_depth_image_topic',       default_value='/aeirobot/vslam_depth',            description='ZED 2i 뎁스 이미지 토픽'),
        DeclareLaunchArgument('zed_left_camera_info_topic',  default_value='/aeirobot/vslam_left_camera_info', description='ZED 2i 카메라 정보 토픽'),

        # Orbbec 336L RGB-D 토픽
        DeclareLaunchArgument('orbbec_color_image_topic',    default_value='/camera/color/image_raw',  description='Orbbec 336L 컬러(또는 mono) 이미지 토픽'),
        DeclareLaunchArgument('orbbec_depth_image_topic',    default_value='/camera/depth/image_raw',  description='Orbbec 336L 뎁스 이미지 토픽'),
        DeclareLaunchArgument('orbbec_camera_info_topic',    default_value='/camera/color/camera_info',description='Orbbec 336L 카메라 정보 토픽'),

        # Image topic compression
        DeclareLaunchArgument('compressed',            default_value='false', description=''),
        DeclareLaunchArgument('rgb_image_transport',   default_value='compressed', description=''),
        DeclareLaunchArgument('depth_image_transport', default_value='compressedDepth', description=''),

        # Odometry
        DeclareLaunchArgument('visual_odometry',            default_value='false',  description=''),
        DeclareLaunchArgument('icp_odometry',               default_value='false',  description=''),
        DeclareLaunchArgument('odom_topic',                 default_value='/zed_odom',  description=''),
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
        DeclareLaunchArgument('imu_topic',        default_value='/zed_imu', description=''),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false',         description=''),

        OpaqueFunction(function=launch_setup)
    ])
