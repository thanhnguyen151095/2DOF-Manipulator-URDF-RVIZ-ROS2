# 2DOF-Manipulator-URDF-RVIZ-ROS2
This tutorial illustrates the process of creating a URDF file for a basic 2-DOF robot manipulator and subsequently verifying it utilizing RVIZ2 in ROS2 Jazzy.


Step 1: The first step is to create a simple 2-DOF robotic arm. This is done using a URDF (Unified Robot Description Format) file. This XML-based format is a standard in ROS and describes the robot's links (its parts), joints (how they move), and visual appearance. 

Our robot will have:
- A base_link: The stationary foundation.
- link1: The first arm segment.
- joint1: A revolute (rotating) joint connecting the base to link1.
- link2: The second arm segment.
- joint2: Another revolute joint connecting link1 to link2.

Create a file named **`two_dof_arm.urdf`**. Later, we will place this inside our ROS2 package.<?xml version="1.0"?>
```bash
<!-- This is the URDF definition for a simple 2-DOF robotic arm -->
<robot name="two_dof_arm">

  <!-- *********************** LINKS *********************** -->
  <!-- Links are the rigid parts of the robot. -->

  <!-- 1. The Base Link (The foundation) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
    </collision>
  </link>

  <!-- 2. The First Arm Link (The "upper arm") -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <material name="orange">
        <color rgba="1.0 0.4 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.1 0.1"/>
      </geometry>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
    </collision>
     <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- 3. The Second Arm Link (The "forearm") -->
  <link name="link2">
    <visual>
      <geometry>
        <box size="0.4 0.08 0.08"/>
      </geometry>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.1 0.1 1.0 1.0"/>
      </material>
    </visual>
    <collision>
       <geometry>
        <box size="0.4 0.08 0.08"/>
      </geometry>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
    </collision>
     <inertial>
        <mass value="0.8"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- *********************** JOINTS *********************** -->
  <!-- Joints connect the links and define how they can move. -->

  <!-- 1. The First Joint (Connects base_link to link1) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>

  <!-- 2. The Second Joint (Connects link1 to link2) -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>

</robot>
```


Step 2 (ROS2 Jazzy): Bringing Your Robot to Life (ament_python) 🚀 We've built the 2-DOF robot (URDF). Now, let's bring it into the ROS2 world using a pure Python package. This involves creating a ROS2 ament_python package, writing a **`setup.py`** file to handle installation, and using our Python launch file to visualize the robot. 

1. Create a ROS2 Python Package

First, we need a home for our robot's files.

Open your terminal and navigate to the src directory of your ROS2 workspace (e.g., **`cd ~/ros2_ws/src`**). Then, run the following command to create a Python-based package:
```bash
ros2 pkg create --build-type ament_python two_dof_arm_description
```
This command creates a new package with a **`setup.py`** and **`package.xml`** file, which is the standard for ament_python packages.

Now, let's create our urdf and launch folders inside the new package:
```bash
cd two_dof_arm_description
mkdir urdf
mkdir launch
mkdir rviz
```
Move your **`two_dof_arm.urdf`** file (from Step 1) into the **`urdf`** directory.

2. Configure the Package Files

ament_python packages use **`setup.py`** to describe how they should be built and installed.

A. Edit package.xml:

Open **`package.xml`** and add these lines to declare the packages we will depend on when the code is executed:  
```bash
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>joint_state_publisher_gui</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>xacro</exec_depend>
```
B. Edit setup.py:

Open **`setup.py`** and replace its contents with the following. This file tells ROS2 how to install your package and, crucially, where to find your urdf, launch, and new .rviz files.
```bash
import os
from glob import glob
from setuptools import setup

package_name = 'two_dof_arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files and rviz files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        # Include all URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Include all rviz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.com',
    description='Description of the 2-DOF arm package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

3. Create the RViz Configuration File
Create a new file named **`view_robot.rviz`** inside your **`rviz`** directory and paste the exact configuration you provided into it:
```bash
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
    - Alpha: 0.8
      Class: rviz_default_plugins/RobotModel
      Description Topic:
        Value: /robot_description
      Name: RobotModel
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Value: true
  Global Options:
    Fixed Frame: base_link
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.7
      Name: Current View
      Pitch: 0.33
      Value: Orbit (rviz)
      Yaw: 5.5
Window Geometry:
  Height: 800
  Width: 1200
```
4. The Python Launch File (Updated)

Now, we update the launch file to use our new **`.rviz`** configuration. Open **`view_robot.launch.py`** and modify it to look like this:
```bash
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
    rviz_config_file = os.path.join(package_dir, 'launch', 'view_robot.rviz')

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

    # ** FIX ** Add the rviz_config_file as an argument
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
 ```   
5. Build and Launch!

- After saving all the changes, rebuild your workspace and launch.
- Build your workspace: colcon build --packages-select two_dof_arm_descriptionSource the workspace: source install/setup.bash
- Run the launch file: ros2 launch two_dof_arm_description view_robot.launch.py

Now, RViz2 should launch with your exact configuration, showing the robot, grid, and TF frames immediately without any manual setup required.
