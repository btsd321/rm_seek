import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler

def generate_launch_description():
    
    share_directory = get_package_share_directory('seek_simu_camera')
    print("seek_simu_camera package share directory: ", share_directory)
    
    # rviz_config_file = LaunchConfiguration('rviz_config') # RViz配置文件相对路径
    sensor_yaml_config_file = LaunchConfiguration('yaml_config') # RViz配置文件相对路径
    
    declare_yaml_config_file_cmd = DeclareLaunchArgument(
        'yaml_config',
        default_value = os.path.join(share_directory, 'config', 'seek_simu_camera.yaml'),
        description='Full path to the YAML config file to use')
    
    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     'rviz_config',
    #     default_value = os.path.join(share_directory, 'rviz', 'seek_simu_camera.rviz'),
    #     description='Full path to the RVIZ config file to use')
    
    start_seek_simu_camera_cmd = Node(
        package='seek_simu_camera',                         # Robot Sensor软件包
        executable='seek_simu_camera_node',                      # 可执行文件为seek_simu_camera_node
        name='seek_simu_camera_node',                          # 节点名称为seek_simu_camera  
        parameters=[sensor_yaml_config_file])                            
    
    # 启动RViz
    # start_rviz_cmd = Node(
    #     package='rviz2',                         # RViz2软件包
    #     executable='rviz2',                      # 可执行文件为rviz2
    #     arguments=['-d', rviz_config_file],
    #     output='screen',)                        # 输出到屏幕
    
    
    # # 退出事件处理器
    # exit_event_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=start_rviz_cmd,       # 关联的目标动作是启动RViz
    #         on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))  # 退出时触发关闭事件
    
    # 创建启动描述并填充
    ld = LaunchDescription()
    ld.add_action(declare_yaml_config_file_cmd)  # 添加YAML配置文件参数
    # ld.add_action(declare_rviz_config_file_cmd)  # 添加RViz配置文件参数
    ld.add_action(start_seek_simu_camera_cmd)  # 启动Robot Sensor
    # ld.add_action(start_rviz_cmd)         # 启动RViz
    # ld.add_action(exit_event_handler)      # 添加退出事件处理器
    return ld  # 返回构建好的启动描述

if __name__ == '__main__':
    generate_launch_description()