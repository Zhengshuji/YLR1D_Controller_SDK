import os
from launch import LaunchDescription
from launch_ros.actions import Node

# 如果未来需要包含其他启动文件，可以取消注释以下导入
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_path

from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    bringup_path = get_package_share_path('bringup')
    rviz_config_file = "default.rviz"
    rviz_config_path = os.path.join(bringup_path, f'config/{rviz_config_file}')

    # 环境参数设置
    env = os.environ.copy()
    env['LIBGL_ALWAYS_SOFTWARE'] = '1'

    # 启动节点示例（注释掉，保留作为参考）
    # Node(
    #     package='',  # 您的C++节点所在的功能包名
    #     executable='',   # 编译后的可执行文件名
    #     name='',
    #     output='screen',
    # )

    # 启动子launch文件示例（注释掉）
    # IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(str(sub_launch_path)),
    #     launch_arguments={
    #         'parameter_name': 'parameter_value',
    #     }.items()
    # )

    camDemoNode = Node(
        package='robot_package',
        executable='camDemo',
        name='camDemo',
        output='screen'
    )

    armDemoNode = Node(
        package='robot_package',
        executable='armDemo',
        name='armDemo',
        output='screen'
    )

    # rviz2节点启动
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[rviz_config_path],
        env=env,
    )


    ld = LaunchDescription()

    ld.add_action(armDemoNode)

    ld.add_action(camDemoNode)
    
    ld.add_action(rviz2_node)

    return ld