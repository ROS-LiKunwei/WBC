import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('sysmo_description')
    config_pkg_dir = get_package_share_directory('fa_moveit2_config')
    ik_pkg_dir = get_package_share_directory('ik_7dof')
    
    # 声明参数
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_dir, 'urdf', 'fa_robot.urdf'),
        description='URDF file path'
    )
    
    srdf_file_arg = DeclareLaunchArgument(
        'srdf_file',
        default_value=os.path.join(config_pkg_dir, 'config', 'fa_robot.srdf'),
        description='SRDF file path'
    )
    
    num_tests_arg = DeclareLaunchArgument(
        'num_tests',
        default_value='5',
        description='Number of test iterations'
    )
    
    max_iters_arg = DeclareLaunchArgument(
        'max_iters',
        default_value='200',
        description='Maximum IK iterations'
    )
    
    eps_arg = DeclareLaunchArgument(
        'eps',
        default_value='1e-3',
        description='Convergence tolerance'
    )
    
    move_delay_arg = DeclareLaunchArgument(
        'move_delay',
        default_value='1.0',
        description='Delay between moves in seconds'
    )
    
    # 包含机器人状态发布器
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # 包含 Rviz 启动文件
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, 'launch', 'moveit_rviz.launch.py')
        )
    )
    
    # 创建IK节点
    ik_rviz_node = Node(
        package='ik_7dof',
        executable='arm_ik_rviz_node',
        name='arm_ik_rviz_node',
        output='screen',
        parameters=[{
            'urdf_file': LaunchConfiguration('urdf_file'),
            'srdf_file': LaunchConfiguration('srdf_file'),
            'num_tests': LaunchConfiguration('num_tests'),
            'max_iters': LaunchConfiguration('max_iters'),
            'eps': LaunchConfiguration('eps'),
            'move_delay': LaunchConfiguration('move_delay'),
        }]
    )
    
    return LaunchDescription([
        urdf_file_arg,
        srdf_file_arg,
        num_tests_arg,
        max_iters_arg,
        eps_arg,
        move_delay_arg,
        rsp_launch,
        rviz_launch,
        ik_rviz_node
    ])