#
# Orbbec Gemini 330 시리즈 (330L/336L) + RTAB-Map
#
# rtabmap_with_gui.launch.py 구조를 따르되 orbbec RGB-D 입력에 맞게 변경:
#   1. 스테레오 입력 → RGB-D 입력 (subscribe_depth)
#   2. 외부 odom(/odometry/filtered) → rtabmap_odom rgbd_odometry (VO) 노드 추가
#   3. 카메라 드라이버(gemini_330_series.launch.py)를 depth_registration:=true 로 포함 실행
#   4. localization argument로 매핑/위치추정 모드 선택 (GUI 서비스 전환 대신 런치 인자)
#   5. force_3dof argument: 로봇 탑재 시 true, 손으로 들고 테스트 시 false
#
# 사용법:
#   매핑:     ros2 launch rtabmap_launch orbbec_rtabmap.launch.py
#   위치추정: ros2 launch rtabmap_launch orbbec_rtabmap.launch.py localization:=true
#
# 토픽 (camera_name=camera 기준):
#   rgb   : /camera/color/image_raw
#   depth : /camera/depth/image_raw   (depth_registration:=true → color에 정렬됨)
#   info  : /camera/color/camera_info
#

import os

# ROS_WS 환경변수 필수
ROS_WS = os.environ.get('ROS_WS')
if not ROS_WS:
    raise RuntimeError("ROS_WS 환경변수가 설정되지 않았습니다. 예: export ROS_WS=/path/to/ws")
FEATURE_EXTRACTORS = os.path.join(ROS_WS, 'src', 'alice_navigation', 'localization', 'feature_extractors')

# 맵 DB 저장 위치 (기존 slam_manager 워크플로와 동일한 ~/.ros/mapping)
MAP_DIR = os.path.expanduser('~/.ros/mapping')
os.makedirs(MAP_DIR, exist_ok=True)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    localization = LaunchConfiguration('localization')
    force_3dof = LaunchConfiguration('force_3dof')

    # 카메라가 sensor_data(Best Effort) QoS로 발행하므로 맞춰줌
    # rgbd_odometry는 이미지 QoS를 'qos'로, rtabmap/viz는 'qos_image'로 받음
    common_params = {
        'frame_id': LaunchConfiguration('frame_id'),
        'subscribe_depth': True,
        # bag 재생 검증 시 false (bag에 odom_info 미수록 — true면 동기화 무한 대기)
        'subscribe_odom_info': LaunchConfiguration('subscribe_odom_info'),
        'approx_sync': LaunchConfiguration('approx_sync'),
        'wait_for_transform': LaunchConfiguration('wait_for_transform'),
        'topic_queue_size': LaunchConfiguration('topic_queue_size'),
        'sync_queue_size': LaunchConfiguration('sync_queue_size'),
        'qos': 2,
        'qos_image': 2,
        'qos_camera_info': 2,
    }

    remappings = [
        ('rgb/image', LaunchConfiguration('rgb_topic')),
        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        ('depth/image', LaunchConfiguration('depth_topic')),
        ('imu', LaunchConfiguration('imu_topic')),
    ]

    return [
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),

        ### IMU orientation 필터 (orbbec IMU는 raw만 발행 — rtabmap은 orientation 필요) ###
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter', output='screen',
            parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/gyro_accel/sample'),
                        ('imu/data', LaunchConfiguration('imu_topic'))],
            condition=IfCondition(LaunchConfiguration('use_imu'))),

        ### Visual Odometry (rtabmap 제공 RGB-D VO — 기본 GFTT, SuperPoint는 rtabmap 노드 전용) ###
        Node(
            condition=IfCondition(LaunchConfiguration('launch_odometry')),
            package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
            parameters=[dict(common_params, **{
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                'Reg/Force3DoF': ParameterValue(force_3dof, value_type=str),
                # 상실 시 즉시 리셋: 발산한 가짜 궤적이 지속되며 rtabmap 메모리를
                # 폭주시킨 사고(setup 3.29) 재발 방지 — 리셋 후 재위치추정으로 복구
                'Odom/ResetCountdown': '1',
            })],
            remappings=remappings,
            # 로거는 네임스페이스 포함 이름(rtabmap.rgbd_odometry) — 노드명만 쓰면 무효과 (2026-07-23 실측)
            arguments=["--ros-args", "--log-level",
                       [LaunchConfiguration('namespace'), '.rgbd_odometry:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        ### Visual SLAM (with_gui와 동일하게 3초 지연 시작) ###
        TimerAction(period=3.0, actions=[
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
            parameters=[dict(common_params, **{
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'publish_tf': LaunchConfiguration('publish_tf_map'),
                'initial_pose': LaunchConfiguration('initial_pose'),
                'database_path': LaunchConfiguration('database_path'),
                'odom_sensor_sync': LaunchConfiguration('odom_sensor_sync'),
                'config_path': LaunchConfiguration('cfg').perform(context),
                'odom_correction': LaunchConfiguration('odom_correction'),

                # ============================================================
                # 모드 선택: localization argument (with_gui는 GUI 서비스로 전환)
                # ParameterValue(str) 필수: substitution 결과는 yaml 타입추론으로
                # bool이 되어 rtabmap의 string 선언과 충돌
                # ============================================================
                'Mem/IncrementalMemory': ParameterValue(
                    PythonExpression(["'false' if '", localization, "' == 'true' else 'true'"]), value_type=str),
                # WM 노드 수 상한(0=무제한). VO 발산 등으로 전역을 가상 이동해도
                # WM 캐시가 맵 전체로 퍼지지 못하게 하는 안전벨트 (setup 3.29)
                'Rtabmap/MemoryThr': ParameterValue(LaunchConfiguration('memory_thr'), value_type=str),
                'Mem/InitWMWithAllNodes': ParameterValue(
                    PythonExpression(["'true' if '", localization, "' == 'true' else 'false'"]), value_type=str),

                'Mem/RecentWmRatio': '0.3',

                # Grid Map 높이 필터링 파라미터 (바닥면 제거용)
                'Grid/MinGroundHeight': '0.0',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '1.0',
                'Grid/NormalsSegmentation': 'false',
                'Grid/RayTracing': 'true',

                'Grid/3D': 'false',
                # 3DoF 강제는 평면 주행 로봇 전용. 손으로 들고 테스트하면 pitch/roll/z가
                # 깎여서 이동량이 노드 생성 임계값에 못 미쳐 맵이 안 자람 (WM=1 고정 증상)
                'Reg/Force3DoF': ParameterValue(force_3dof, value_type=str),
                'RGBD/ForceOdom3DoF': ParameterValue(force_3dof, value_type=str),
                'Reg/Strategy': '0',
                'Kp/DetectorStrategy': '11',
                'Vis/FeatureType': '11',
                'Vis/MaxFeatures': '1000',
                'ORB/Gpu': 'false',
                'Mem/ImagePostDecimation': '1',
                'Mem/ImagePreDecimation': '1',
                'SuperPoint/ModelPath': os.path.join(FEATURE_EXTRACTORS, 'superpoint_v1.pt'),
                'PyMatcher/Path': os.path.join(FEATURE_EXTRACTORS, 'SuperGluePretrainedNetwork', 'rtabmap_superglue.py'),
                'Vis/CorGuessWinSize': '0',
                'Vis/CorNNType': '6',
                'Reg/RepeatOnce': 'false',
                'Kp/MaxFeatures': '1000',
                'Mem/RehearsalSimilarity': '0.6',
                'RGBD/OptimizeMaxError': '3.0',
                'Optimizer/Robust': 'false',
                # LC rate [Hz] — SuperPoint/SuperGlue가 GPU를 쓰므로 작은 GPU에서 세만틱과 경합 시 낮출 것
        'Rtabmap/DetectionRate': ParameterValue(LaunchConfiguration('detection_rate'), value_type=str),
                'Kp/MaxDepth': '15.0',
                'Vis/MinInliers': '30',
                'Vis/PnPReprojError': '4',
                'RGBD/ProximityPathFilteringRadius': '2.0',
                'RGBD/ProximityMaxGraphDepth': '100',
                'RGBD/LocalRadius': '15.0',
                'Rtabmap/LoopThr': '0.11',
                'RGBD/MaxOdomCacheSize': '10',

                ## 레전드 파라미터 ##
                'SuperPoint/Threshold': '0.005',
                'SuperPoint/NMSRadius': '4',
                'PyMatcher/Iterations': '40',
                'PyMatcher/Threshold': '0.2',
                'PyMatcher/Model': 'indoor',

                ## Prior 워크플로 (with_gui와 동일) ##
                'Optimizer/PriorsIgnored': 'false',
                'Optimizer/Strategy': '2',            # GTSAM
                'Optimizer/Iterations': '100',
                'RGBD/OptimizeFromGraphEnd': 'false',
                'RGBD/StartAtOrigin': 'true',
                'Optimizer/LandmarksIgnored': 'false',
            })],
            remappings=remappings + [('map', LaunchConfiguration('map_topic'))],
            # 매핑 모드는 -d(기존 DB 삭제 후 새로 시작), localization 모드는 DB 유지·로드
            arguments=[
                LaunchConfiguration('rtabmap_args'),
                PythonExpression(["'' if '", localization, "' == 'true' else '-d'"]),
                "--ros-args", "--log-level",
                [LaunchConfiguration('namespace'), '.rtabmap:=', LaunchConfiguration('log_level')]],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),
        ]),  # TimerAction 닫기

        ### Rtabmap GUI ###
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz', output='screen',
            parameters=[common_params],
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            arguments=[LaunchConfiguration('gui_cfg')],
            prefix=LaunchConfiguration('launch_prefix'),
            namespace=LaunchConfiguration('namespace')),

        ### Rviz ###
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=[['-d'], [LaunchConfiguration('rviz_cfg')]]),
    ]


def generate_launch_description():

    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )

    return LaunchDescription([

        # Arguments
        ## 모드 선택
        DeclareLaunchArgument('localization', default_value='false', description='true: 기존 DB로 위치추정 모드, false: 매핑 모드'),
        DeclareLaunchArgument('force_3dof',   default_value='false', description='true: 평면(3DoF) 강제 — 로봇 탑재 시 사용. 손으로 들고 테스트할 땐 false'),
        DeclareLaunchArgument('detection_rate', default_value='2', description='loop closure 감지율 [Hz]. GPU 경합 시(라이프롱 스택 노트북 구동) 1 권장'),
        DeclareLaunchArgument('memory_thr', default_value='0',
                              description='Rtabmap/MemoryThr — WM 노드 수 상한(0=무제한). 장시간 localization 운영 시 350 권장 (setup 3.29)'),

        ## GUI ON / OFF
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI (optional).'),
        DeclareLaunchArgument('rviz',        default_value='true', description='Launch RVIZ (optional).'),

        ## odom tf 보정
        DeclareLaunchArgument('odom_correction', default_value='true', description='loop closing 상황에서 odom tf 옮길건지 말건지 선택하는 변수'),

        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('launch_odometry', default_value='true',
                              description='rgbd_odometry 실행 여부. bag 재생 검증(odom이 bag에 있음)이면 false'),
        DeclareLaunchArgument('subscribe_odom_info', default_value='true',
                              description='odom_info 구독. bag 재생 검증(미수록)이면 false'),
        DeclareLaunchArgument('log_level',    default_value='info', description="ROS logging level (debug, info, warn, error)."),

        # Config files
        DeclareLaunchArgument('cfg',      default_value='',                        description='RTAB-Map config file path (*.ini).'),
        DeclareLaunchArgument('gui_cfg',  default_value='~/.ros/rtabmap_gui.ini',  description='Configuration path of rtabmap_viz.'),
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz,               description='Configuration path of rviz2.'),

        DeclareLaunchArgument('frame_id',       default_value='base_footprint',    description='기준 프레임 (카메라 런치가 base_footprint->camera_link tf 발행)'),
        DeclareLaunchArgument('map_frame_id',   default_value='map',               description=''),
        DeclareLaunchArgument('map_topic',      default_value='map',               description=''),
        DeclareLaunchArgument('publish_tf_map', default_value='true',              description=''),
        DeclareLaunchArgument('namespace',      default_value='rtabmap',           description=''),
        DeclareLaunchArgument('database_path',  default_value=os.path.join(MAP_DIR, 'orbbec_rtabmap.db'),
                              description='맵 DB 경로 (매핑 모드는 시작 시 삭제 후 새로 생성, localization 모드는 로드). 예: database_path:=~/.ros/mapping/field_x.db'),
        DeclareLaunchArgument('topic_queue_size', default_value='10',              description=''),
        DeclareLaunchArgument('sync_queue_size',  default_value='10',              description=''),
        DeclareLaunchArgument('wait_for_transform', default_value='0.2',           description=''),
        DeclareLaunchArgument('rtabmap_args',   default_value='',                  description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug'),
        DeclareLaunchArgument('launch_prefix',  default_value='',                  description=''),
        DeclareLaunchArgument('initial_pose',   default_value='',                  description=''),
        DeclareLaunchArgument('odom_sensor_sync', default_value='false',           description=''),

        DeclareLaunchArgument('approx_sync', default_value='true', description='rgb/depth 근사 시간동기화 (enable_frame_sync 완벽하면 false 가능)'),

        # RGB-D 토픽 (orbbec 카메라, camera_name=camera 기준)
        DeclareLaunchArgument('rgb_topic',         default_value='/camera/color/image_raw',   description=''),
        DeclareLaunchArgument('depth_topic',       default_value='/camera/depth/image_raw',   description=''),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info', description=''),

        # imu (use_imu:=true → 카메라 내장 IMU 활성화 + madgwick 필터로 orientation 추정 → VO 중력 정렬)
        DeclareLaunchArgument('use_imu',          default_value='true',         description='카메라 내장 IMU를 VO 중력 정렬에 사용 (imu_filter_madgwick 패키지 필요)'),
        DeclareLaunchArgument('imu_topic',        default_value='/rtabmap/imu',  description='orientation이 채워진 IMU 토픽 (madgwick 필터 출력)'),
        DeclareLaunchArgument('wait_imu_to_init', default_value=LaunchConfiguration('use_imu'), description=''),

        # 카메라 드라이버
        DeclareLaunchArgument('launch_camera', default_value='true', description='카메라 드라이버 포함 실행 여부 (이미 켜져 있으면 false)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('orbbec_camera'), 'launch'),
                '/gemini_330_series.launch.py']),
            launch_arguments={'depth_registration': 'true',
                              'enable_frame_sync': 'true',
                              # 640x480@15 고정: 네이티브(1280x800)는 픽셀 3.3배라 VO 단일스레드가
                              # 코어 포화로 3.9Hz까지 붕괴 (2026-07-22 풀스택 실측). 해상도가 최대 지렛대
                              'color_width': '640', 'color_height': '480', 'color_fps': '15',
                              'depth_width': '640', 'depth_height': '480', 'depth_fps': '15',
                              # depth 노이즈 필터 (실기 TSDF 메시 품질 — 시뮬 대비 괴리 완화)
                              'enable_spatial_filter': 'true',
                              'enable_temporal_filter': 'true',
                              # Reliable 발행: rtabmap(Best Effort 구독)과 Khronos(Reliable 구독) 모두 호환.
                              # sensor_data(Best Effort) 기본값이면 Khronos가 camera_info를 못 받아 초기화에 갇힘
                              'color_qos': 'default',
                              'depth_qos': 'default',
                              'color_camera_info_qos': 'default',
                              'depth_camera_info_qos': 'default',
                              'enable_accel': LaunchConfiguration('use_imu'),
                              'enable_gyro': LaunchConfiguration('use_imu'),
                              'enable_sync_output_accel_gyro': LaunchConfiguration('use_imu')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),

        OpaqueFunction(function=launch_setup)
    ])
