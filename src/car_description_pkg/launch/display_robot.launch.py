import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions

def generate_launch_description():
    urdf_pkg_path = get_package_share_directory("car_description_pkg")
    default_model_path = '/urdf/first_robot.urdf'   ###允许传入model替换默认的相对路径

    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',default_value=str(default_model_path),
        description='URDF的相对路径'
    )

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ',str(urdf_pkg_path),launch.substitutions.LaunchConfiguration('model')]),value_type=str)
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable = 'joint_state_publisher',    )    ###关节通过话题实时发布关节状态，所以不需要传入parameters
    
    riviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
    )
    
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        riviz_node
    ])
