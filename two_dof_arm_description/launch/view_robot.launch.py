import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():


    # Get the package share directory
    package_dir = get_package_share_directory('two_dof_arm_description')

    # Get the path to the URDF file
    urdf_file_path = os.path.join(package_dir, 'urdf', 'two_dof_arm.urdf')

    # Process the URDF file
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()

    # Get the path to the RViz config file
    rviz_config_file = os.path.join(package_dir, 'rviz', 'view_robot.rviz')



    # Create the necessary nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'robot_description': robot_description_raw}]
    )

    #  Add the rviz_config_file as an argument
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])