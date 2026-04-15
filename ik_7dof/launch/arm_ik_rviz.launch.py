import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 引入 MoveIt 2 专属的参数构建器
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg_dir = get_package_share_directory('sysmo_description')
    config_pkg_dir = get_package_share_directory('fa_moveit2_config')
    
    # 声明自定义参数
    urdf_file_arg = DeclareLaunchArgument('urdf_file', default_value=os.path.join(pkg_dir, 'urdf', 'fa_robot.urdf'))
    srdf_file_arg = DeclareLaunchArgument('srdf_file', default_value=os.path.join(config_pkg_dir, 'config', 'fa_robot.srdf'))
    num_tests_arg = DeclareLaunchArgument('num_tests', default_value='10')
    max_iters_arg = DeclareLaunchArgument('max_iters', default_value='1000')
    eps_arg = DeclareLaunchArgument('eps', default_value='1e-3')
    move_delay_arg = DeclareLaunchArgument('move_delay', default_value='1.0')
    trajectory_duration_arg = DeclareLaunchArgument('trajectory_duration', default_value='2.0')
    arm_side_arg = DeclareLaunchArgument('arm_side', default_value='left')

    # ✨ 核心修复 1：使用 Builder 自动抓取 config 包里的所有 MoveIt 参数（URDF, SRDF, Kinematics等）
    # 注意：这里的 "fa_robot" 是指你的包内部文件前缀，如果不匹配，请填入正确的机器人名称
    moveit_config = MoveItConfigsBuilder("fa_robot", package_name="fa_moveit2_config").to_moveit_configs()

    # ✨ 核心修复 2：直接包含 config 包的 demo.launch.py，它会完美启动 move_group 和 rviz
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, 'launch', 'demo.launch.py')
        ),
        # 强制开启伪造执行，这样 MoveIt 就会在内部模拟轨迹执行并在 RViz 中显示
        launch_arguments={'use_fake_hardware': 'true'}.items()
    )
    
    # 创建你自己的 IK 节点
    ik_rviz_node = Node(
        package='ik_7dof',
        executable='arm_ik_rviz_node',
        name='arm_ik_rviz_node',
        output='screen',
        parameters=[
            # ✨ 核心修复 3：将庞大的 MoveIt 配置字典强行注入到你的 C++ 节点中
            # 只有这样，你代码里的 MoveGroupInterface 才能正常初始化！
            moveit_config.to_dict(),
            
            # 你的自定义参数
            {
                'urdf_file': LaunchConfiguration('urdf_file'),
                'srdf_file': LaunchConfiguration('srdf_file'),
                'num_tests': LaunchConfiguration('num_tests'),
                'max_iters': LaunchConfiguration('max_iters'),
                'eps': LaunchConfiguration('eps'),
                'move_delay': LaunchConfiguration('move_delay'),
                'trajectory_duration': LaunchConfiguration('trajectory_duration'),
                'arm_side': LaunchConfiguration('arm_side'),
            }
        ]
    )
    
    return LaunchDescription([
        urdf_file_arg,
        srdf_file_arg,
        num_tests_arg,
        max_iters_arg,
        eps_arg,
        move_delay_arg,
        trajectory_duration_arg,
        arm_side_arg,
        
        # 启动整个 MoveIt 环境
        demo_launch,
        # 启动你的测试节点
        ik_rviz_node
    ])