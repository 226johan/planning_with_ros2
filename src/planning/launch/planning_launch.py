from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # get path
    planning_path = get_package_share_directory('planning')


    # get main_car urdf
    car_path = os.path.join(
        planning_path,"urdf/main_car","car.xacro"
    )

    # get rviz path
    rviz_conf_path = os.path.join(planning_path,'rviz','planning.rviz')

    # run  urdf commond
    car_para = ParameterValue(Command(["xacro ",car_path]))

    # robot state publisher node
    car_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="car_state_pub",
        output="screen",
        parameters=[{"robot_description":car_para}],
    )

    # joint_state_publisher node


    car_joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="car_joint_state_gui",
    )
    car_joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="car_joint_state_pub",
    )

    # rviz node
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d",rviz_conf_path],
    )

    # 绘图节点

    # 启动地图服务器节点

    # 启动全局路径服务器节点

    # 启动规划器节点
    planning_process = Node(
        package="planning",
        executable="planning_process",
        name="planning_process",
    )

    # 节点分组
    car_main = GroupAction(
        actions=[
            PushRosNamespace("car"),
            car_state_pub,
            car_joint_state_pub,
        ]
    )

    planning = GroupAction(
        actions=[
            PushRosNamespace("planning"),
            planning_process,
        ]
    )

    return LaunchDescription(
        [
            car_main,
            planning,
            rviz2
        ]
    )


