import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('humanoid_description')
    config_pkg_dir = get_package_share_directory('humanoid_moveit2_config')
    # 声明参数
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_dir, 'urdf', 'humanoid.urdf'),
        description='URDF file path'
    )
    
    srdf_file_arg = DeclareLaunchArgument(
        'srdf_file',
        default_value=os.path.join(config_pkg_dir, 'config', 'humanoid.srdf'),
        description='SRDF file path'
    )
    
    num_tests_arg = DeclareLaunchArgument(
        'num_tests',
        default_value='10000',
        description='Number of test iterations'
    )
    
    max_iters_arg = DeclareLaunchArgument(
        'max_iters',
        default_value='1000',
        description='Maximum IK iterations'
    )
    
    eps_arg = DeclareLaunchArgument(
        'eps',
        default_value='1e-3',
        description='Convergence tolerance'
    )
    
    # 创建节点
    ik_test_node = Node(
        package='ik_7dof',
        executable='left_arm_ik_node',
        name='left_arm_ik_node',
        output='screen',
        parameters=[{
            'urdf_file': LaunchConfiguration('urdf_file'),
            'srdf_file': LaunchConfiguration('srdf_file'),
            'num_tests': LaunchConfiguration('num_tests'),
            'max_iters': LaunchConfiguration('max_iters'),
            'eps': LaunchConfiguration('eps'),
        }]
    )
    
    return LaunchDescription([
        urdf_file_arg,
        srdf_file_arg,
        num_tests_arg,
        max_iters_arg,
        eps_arg,
        ik_test_node
    ])