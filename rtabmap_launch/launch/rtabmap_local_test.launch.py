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
        
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
              
        
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
                # "Mem/IncrementalMemory": ConditionalText("true", "false", IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' != 'true'"]))._predicate_func(context)).perform(context),
                # "Mem/InitWMWithAllNodes": ConditionalText("true", "false", IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), "' == 'true'"]))._predicate_func(context)).perform(context),
                
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
    
    
                'Mem/RecentWmRatio': '0.3',   # 기본값 0.2, 마지막 루프 클로징을 기준으로 WM에 20%만 냅두고 나머지는 LTM으로 옮기겠다는 의미, LTM으로 들어가면 해당 노드는 루프 클로징에 사용되지 않음. -> 수십GB급 맵 데이터에서도 실시간성 유지를 위해 0.2였던 것, 테스트 해보면서 값 조절하기
                "odom_correction": LaunchConfiguration('odom_correction'),
                # "Grid/MaxObstacleHeight": "0.9",
                'Grid/3D': "false",  # 2D Occupancy Grid 활성화
                "Reg/Force3DoF": "true",
                'RGBD/ForceOdom3DoF': 'true',      # VO 추정 z/roll/pitch 를 0으로 강제 => 기본값은 true
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
                'Vis/DepthMaskFloorThr': '0.4',   # 뎁스값 추정해서 바닥은 필터링해서 없애는 거 ...아직 잘 모르겠음ㅠ
                'Kp/MaxFeatures': '500',
                'Mem/STMSize': '15',               # 10 -> 맵핑할 때 WM의 최소 갯수를 의미함. 10개 이상 모여야 루프 클로징 시작
                'Mem/RehearsalSimilarity': '0.6',  # 0.6
                'RGBD/OptimizeMaxError': '0.0',    # 기본값: 3.0 | OptimizeMaxError, Robust 는 서로 상반된 파라미터임, OptimizeMaxError 값이 있으면 Robust는 false, OptimizeMaxError = 0.0이면 Robust는 true 가능
                'Optimizer/Robust': 'true',        # 기본값: false | OptimizeMaxError 표준편차 기준으로 그래프에서 멀리 떨어진 루프 클로징을 거절함 -> 빠르다. Robust는 0~1까지의 가중치를 계산해서 루프 클로징을 거절 또는 수락함.-> 느리지만 공장,복도와 같은 대규모 장소에 적합.
                'Rtabmap/DetectionRate': '2',     # 기본값 1, 루프 클로징 rate를 의미 단위[Hz]

                ## 레전드 파라미터 ##
                'SuperPoint/Threshold': '0.005',    # 기본값 0.010
                'SuperPoint/NMSRadius': '2',      # 기본값 4
                'PyMatcher/Iterations': '40',     # 기본값 20
                'PyMatcher/Threshold': '0.15',     # 기본값 0.2
                
                'PyMatcher/Model': 'outdoor',      # indoor outdoor
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
        
        
        ### vslam 결과 publish 해주는 노드 -> localization manager로 넘길 데이터
        Node(
            package='rtabmap_pose_publisher', 
            executable='rtabmap_pose_publisher', 
            name='rtabmap_pose_publisher',
            output='screen',
            parameters=[{
                "correction_base_frame_id": LaunchConfiguration('correction_base_frame_id'),
                "correction_odom_frame_id": LaunchConfiguration('correction_odom_frame_id'),
                'min_loop_score': LaunchConfiguration('min_loop_score'),
                'info_topic': LaunchConfiguration('info_topic'),
                'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
                'map_to_odom_topic': LaunchConfiguration('map_to_odom_topic'),
                'global_pose_topic': LaunchConfiguration('global_pose_topic'),
                }],
            ),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')
        ]

def generate_launch_description():
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )
    
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument('stereo', default_value='true', description='Use stereo input instead of RGB-D.'),

        ## Mapping, Localization, Extend mapping 모드 선택
        ### MAPPING MODE:           localization: false, extend_map: false  ###
        ### EXTEND MAPPING MODE:    localization: false, extend_map: true   ###
        ### LOCALIZATION MODE:      localization: true,  extend_map: false  ###
        DeclareLaunchArgument('localization', default_value='true', description='true면 Localization 모드, false면 mapping모드'),
        DeclareLaunchArgument('extend_map', default_value='false', description='LTM 데이터를 모두 WM로 불러온 상태로 추가 맵핑 진행'),
        
        ## GUI ON / OFF
        DeclareLaunchArgument('rtabmap_viz',  default_value='false',  description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',         default_value='false', description='Launch RVIZ (optional).'),

        ## odom tf 보정 할지말지 변수
        DeclareLaunchArgument('odom_correction', default_value='true', description='loop closing 상황에서 odom tf 옮길건지 말건지 선택하는 변수'),
        
        ### rtabmap_pose_publisher에서 사용하는 변수 ###
        DeclareLaunchArgument('correction_base_frame_id', default_value='pelvis_waist',                            description='localization manager에 vslam 결과 보내주는 용'),
        DeclareLaunchArgument('correction_odom_frame_id', default_value='odom',                                    description='localization manager에 vslam 결과 보내주는 용'),
        DeclareLaunchArgument('min_loop_score',           default_value='0.4',                                     description='루프 클로징 했을 때 local manager로 보내주기 위한 최소 점수, 해당 점수보다 낮으면 루프 클로징 되어도 무시됨'),
        DeclareLaunchArgument('info_topic',               default_value='rtabmap/info',                            description='pose_publisher에서 사용할 토픽 정의'),
        DeclareLaunchArgument('zed_odom_topic',           default_value='/zed_odom',                               description='pose_publisher에서 사용할 토픽 정의'),
        DeclareLaunchArgument('map_to_odom_topic',        default_value='/aeirobot/localization/map_to_odom',      description='pose_publisher에서 사용할 토픽 정의'),
        DeclareLaunchArgument('global_pose_topic',        default_value='/aeirobot/localization/map_to_pelvis_visual_slam', description='3D map -> pelvis pose'),
        DeclareLaunchArgument('global_pose_topic_2d',     default_value='/aeirobot/localization/pose_visual_slam', description='2D map -> pelvis pose'),
        ### rtabmap_pose_publisher에서 사용하는 변수 ###
        
        
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('log_level',    default_value='info', description="ROS logging level (debug, info, warn, error). For RTAB-Map\'s logger level, use \"args\" argument."),


        # Config files  ~/.rtabmap/test.ini
        DeclareLaunchArgument('cfg',      default_value='',                        description='To change RTAB-Map\'s parameters, set the path of config file (*.ini) generated by the standalone app.'),
        DeclareLaunchArgument('gui_cfg',  default_value='~/.ros/rtabmap_gui.ini',  description='Configuration path of rtabmap_viz.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,               description='Configuration path of rviz2.'),

        DeclareLaunchArgument('frame_id',       default_value='pelvis_waist',          description='zed_camera_link  tracker'),
        DeclareLaunchArgument('odom_frame_id',  default_value='odom',                   description='If set, TF is used to get odometry instead of the topic.'),
        DeclareLaunchArgument('map_frame_id',   default_value='map',                description='Output map frame id (TF).'),
        DeclareLaunchArgument('map_topic',      default_value='map',                description='Map topic name.'),
        DeclareLaunchArgument('publish_tf_map', default_value='true',               description='Publish TF between map and odomerty.'),
        DeclareLaunchArgument('namespace',      default_value='rtabmap',            description=''),
        DeclareLaunchArgument('database_path',  default_value='~/.ros/mapping/alicev2.db',  description='Where is the map saved/loaded.'),
        DeclareLaunchArgument('topic_queue_size', default_value='1',                description='Queue size of individual topic subscribers.'),
        DeclareLaunchArgument('queue_size',     default_value='10',                 description='Backward compatibility, use "sync_queue_size" instead.'),
        DeclareLaunchArgument('qos',            default_value='2',                  description='General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('wait_for_transform', default_value='0.2',            description='0.2'),
        DeclareLaunchArgument('rtabmap_args',   default_value='',                   description='Backward compatibility, use "args" instead.'),
        DeclareLaunchArgument('launch_prefix',  default_value='',                   description='For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args"'),
        DeclareLaunchArgument('output',         default_value='screen',             description='Control node output (screen or log).'),
        DeclareLaunchArgument('initial_pose',   default_value='',                   description='Set an initial pose (only in localization mode). Format: "x y z roll pitch yaw" or "x y z qx qy qz qw". Default: see "RGBD/StartAtOrigin" doc'),

        ### 이게 있어야 맵이 받아지는듯?? ###
        DeclareLaunchArgument('ground_truth_frame_id',      default_value='', description='e.g., "world"'),
        DeclareLaunchArgument('ground_truth_base_frame_id', default_value='', description='e.g., "tracker", a fake frame matching the frame "frame_id" (but on different TF tree)'),
        
        DeclareLaunchArgument('approx_sync',  default_value='true',            description='If timestamps of the input topics should be synchronized using approximate or exact time policy.'),
        DeclareLaunchArgument('approx_sync_max_interval',  default_value='1.0', description='(sec) 0 means infinite interval duration (used with approx_sync=true)'),

        # Stereo related topics
        DeclareLaunchArgument('stereo_namespace',        default_value='', description=''),
       
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
        DeclareLaunchArgument('odom_topic',                 default_value='/zed_odom',  description='Odometry topic name., /zed_odom '),
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
               
        OpaqueFunction(function=launch_setup)
    ])
