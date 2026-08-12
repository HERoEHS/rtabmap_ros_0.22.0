#
# ZED(Isaac sim) + rtabmap 부트스트랩 매핑 — orbbec_rtabmap.launch.py 의 zed/sim 프리셋 래퍼
#
# 실물 부트스트랩(orbbec_rtabmap.launch.py 직접 실행)의 sim 대응물:
#   카메라 드라이버/VO/카메라 IMU 없이, Isaac(sim.launch.py)이 발행하는
#   /aeirobot/vslam_left_* 토픽과 EKF odom(/odometry/filtered)을 소비한다.
#
# 사용법 (slam_manager lifelong_zed_mapping 프로세스가 이 형태로 실행):
#   ros2 launch rtabmap_launch zed_rtabmap.launch.py localization:=false database_path:=~/.aeirobot/maps/slam/xxx.db
#
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # zed_sdk_stream:=true 면 aeirobot_zed_camera 노드가 Isaac ZED extension
    # 스트림을 실 ZED SDK 로 처리해서 /aeirobot/vslam_* 발행 (zed_lifelong 과 동일)
    zed_sdk_stream_arg = DeclareLaunchArgument(
        'zed_sdk_stream', default_value='false',
        description='Isaac ZED SDK 스트림 카메라 노드 기동 여부')
    stream_ip_arg = DeclareLaunchArgument(
        'stream_ip', default_value='127.0.0.1',
        description='ZED SDK 스트림 IP (Isaac 실행 머신)')
    stream_port_arg = DeclareLaunchArgument(
        'stream_port', default_value='30000',
        description='ZED SDK 스트림 포트 (Isaac zed_streaming_port)')
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps', default_value='30',
        description='grab FPS (Isaac zed_fps와 일치)')

    zed_sdk_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('aeirobot_zed_camera'),
            'launch', 'camera_img_only.launch.py')),
        condition=IfCondition(LaunchConfiguration('zed_sdk_stream')),
        launch_arguments={
            'stream_ip': LaunchConfiguration('stream_ip'),
            'stream_port': LaunchConfiguration('stream_port'),
            'camera_fps': LaunchConfiguration('camera_fps'),
            'use_sim_time': 'true',
            'use_node_clock_stamp': 'true',
        }.items())

    return LaunchDescription([
        zed_sdk_stream_arg,
        stream_ip_arg,
        stream_port_arg,
        camera_fps_arg,
        zed_sdk_camera,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch', 'orbbec_rtabmap.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'launch_camera': 'false',
                'launch_odometry': 'false',       # VO 대신 EKF odom
                'subscribe_odom_info': 'false',   # 외부 odom 은 odom_info 미발행
                'odom_topic': '/odometry/filtered',
                'use_imu': 'false',               # 카메라 내장 IMU 없음 (madgwick 불필요)
                'wait_imu_to_init': 'false',
                'force_3dof': 'true',             # 평면 주행 로봇 탑재 형상
                # Isaac ZED 발행 토픽 (aeirobot_lifelong zed 프리셋과 동일 값)
                'rgb_topic': '/aeirobot/vslam_left_image',
                'depth_topic': '/aeirobot/vslam_depth',
                'camera_info_topic': '/aeirobot/vslam_left_camera_info',
            }.items()),
    ])
