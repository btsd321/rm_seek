import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions

def generate_launch_description():
    # 获取默认的urdf文件路径
    rm_seek_package_path = get_package_share_directory('rm_seek')
    urdf_file_path = os.path.join(rm_seek_package_path, 'urdf', 'seek_robot.urdf.xacro')
    print('urdf_file_path:', urdf_file_path)
    
    # 获取 seek_simu_camera 的配置文件路径
    seek_simu_camera_package_path = get_package_share_directory('seek_simu_camera')
    seek_simu_camera_config_path = os.path.join(seek_simu_camera_package_path, 'config', 'seek_simu_camera.yaml')

    # 获取 seek_simu_camera 的配置文件路径
    seek_detector_package_path = get_package_share_directory('seek_detector')
    seek_detector_config_path = os.path.join(seek_detector_package_path, 'config', 'seek_detector_node.yaml')
    
    # 获取 seek_solver 的配置文件路径
    seek_solver_package_path = get_package_share_directory('seek_solver')
    seek_solver_config_path = os.path.join(seek_solver_package_path, 'config', 'seek_solver_node.yaml')
    
    # 声明参数
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        'urdf_file_path',
        default_value=str(urdf_file_path),
        description='Path to the URDF file for the robot'
    )
    
    # 读取URDF文件内容
    subsitution_command_result = launch.substitutions.Command([
        'xacro ',
        launch.substitutions.LaunchConfiguration('urdf_file_path')
    ])
    # print(subsitution_command_result)
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        subsitution_command_result
    )
    
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(rm_seek_package_path, 'rviz', 'seek_robot.rviz')],
    )
    
    # 启动 seek_simu_camera_node 节点
    action_seek_simu_camera_node = launch_ros.actions.Node(
        package='seek_simu_camera',
        executable='seek_simu_camera_node',
        name='seek_simu_camera_node',
        parameters=[seek_simu_camera_config_path],  # 加载 YAML 配置文件
    )
    
    # 启动 seek_detector_node 节点
    action_seek_detector_node = launch_ros.actions.Node(
        package='seek_detector',
        executable='seek_detector_node',
        name='seek_detector_node',
        parameters=[seek_detector_config_path],  # 加载 YAML 配置文件
    )
    
    # 启动 seek_solver_node 节点
    action_seek_solver_node = launch_ros.actions.Node(
        package='seek_solver',
        executable='seek_solver_node',
        name='seek_solver_node',
        parameters=[seek_solver_config_path],  # 加载 YAML 配置文件
    )
    
    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_seek_simu_camera_node,
        action_seek_detector_node,
        action_seek_solver_node,
        action_rviz_node
    ])