from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params=str(Path(__file__).resolve().parents[1]/'config/params.yaml')
    clock={'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    nodes=[Node(package='lio_sam',executable='lio_sam_'+part,parameters=[params,clock],output='screen') for part in ['imuPreintegration','imageProjection','featureExtraction','mapOptimization']]
    nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--frame-id','map','--child-frame-id','odom'],parameters=[clock],output='screen'))
    return LaunchDescription([DeclareLaunchArgument('use_sim_time',default_value='false')]+nodes)
