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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch', 'orbbec_rtabmap.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'rviz': 'false',                  # 매핑 감독 GUI 는 rtabmap_viz 로 충분 — rviz2 중복 제거
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
