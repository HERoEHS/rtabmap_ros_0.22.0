#
# Orbbec Gemini 330 시리즈 — lifelong SLAM (P2) 검증용 bag 녹화
#
# 카메라 + rgbd_odometry(VO)만 켜고 Khronos 재생에 필요한 토픽을 전부 기록한다.
# rtabmap SLAM/viz는 녹화에 불필요해서 켜지 않음.
#
# 사용법:
#   ros2 launch rtabmap_launch orbbec_record.launch.py            # 녹화 시작
#   (끝낼 때 Ctrl-C — bag이 자동 마무리됨)
#   ros2 launch rtabmap_launch orbbec_record.launch.py bag_name:=office_test2
#
# 저장 위치: ~/datasets/lifelong/<bag_name>_<YYYYmmdd_HHMMSS>/
#
# 기록 토픽:
#   /camera/color/image_raw, /camera/color/camera_info   (RGB)
#   /camera/depth/image_raw, /camera/depth/camera_info   (depth, color 정렬)
#   /camera/gyro_accel/sample                            (IMU raw, use_imu:=true일 때만 발행됨)
#   /rtabmap/odom                                        (VO odometry)
#   /tf, /tf_static
#
# 해상도 640x480@15fps 고정 (10분 ≈ 14GB). 필요 시 color_width 등 인자로 변경.
#

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BAG_ROOT = os.path.expanduser('~/datasets/lifelong')
os.makedirs(BAG_ROOT, exist_ok=True)
STAMP = datetime.now().strftime('%Y%m%d_%H%M%S')

RECORD_TOPICS = [
    '/camera/color/image_raw',
    '/camera/color/camera_info',
    '/camera/depth/image_raw',
    '/camera/depth/camera_info',
    '/camera/gyro_accel/sample',
    '/rtabmap/odom',
    '/tf',
    '/tf_static',
]


def generate_launch_description():

    bag_path = [LaunchConfiguration('bag_name'), '_' + STAMP]

    return LaunchDescription([

        DeclareLaunchArgument('bag_name', default_value='lifelong_test',
                              description='bag 이름 접두어 (타임스탬프 자동 부착)'),
        DeclareLaunchArgument('use_imu', default_value='false',
                              description='카메라 IMU 활성화 + VO 중력정렬 (imu_filter_madgwick 필요)'),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='녹화 중 컬러 영상 미리보기 창 (rqt_image_view)'),

        # 카메라 드라이버 (depth를 color에 정렬, 640x480@15fps)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('orbbec_camera'), 'launch'),
                '/gemini_330_series.launch.py']),
            launch_arguments={'depth_registration': 'true',
                              'enable_frame_sync': 'true',
                              'color_width': '640', 'color_height': '480', 'color_fps': '15',
                              'depth_width': '640', 'depth_height': '480', 'depth_fps': '15',
                              'enable_accel': LaunchConfiguration('use_imu'),
                              'enable_gyro': LaunchConfiguration('use_imu'),
                              'enable_sync_output_accel_gyro': LaunchConfiguration('use_imu')}.items(),
        ),

        # IMU orientation 필터 (use_imu:=true일 때만)
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter', output='screen',
            parameters=[{'use_mag': False, 'world_frame': 'enu', 'publish_tf': False}],
            remappings=[('imu/data_raw', '/camera/gyro_accel/sample'),
                        ('imu/data', '/rtabmap/imu')],
            condition=IfCondition(LaunchConfiguration('use_imu'))),

        # Visual Odometry (orbbec_rtabmap.launch.py의 VO 설정과 동일)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry', output='screen',
            namespace='rtabmap',
            parameters=[{
                'frame_id': 'base_footprint',
                'approx_sync': True,
                'wait_imu_to_init': LaunchConfiguration('use_imu'),
                'qos': 2,
                'qos_camera_info': 2,
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw'),
                ('imu', '/rtabmap/imu'),
            ]),

        # 녹화 확인용 영상 미리보기 (VO 상태는 터미널의 "Odom: quality=" 로그로 확인)
        Node(
            package='rqt_image_view', executable='rqt_image_view', name='record_monitor',
            arguments=['/camera/color/image_raw'],
            condition=IfCondition(LaunchConfiguration('monitor'))),

        # bag 녹화
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path] + RECORD_TOPICS,
            cwd=BAG_ROOT,
            output='screen'),

        LogInfo(msg=['녹화 시작. 저장 위치: ', BAG_ROOT, '/', LaunchConfiguration('bag_name'), '_' + STAMP,
                     '  (종료: Ctrl-C)']),
    ])
