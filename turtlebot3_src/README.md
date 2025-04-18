# For code to be pulled onto the bot!
- Nav2
- Embedded
- ESC Control
- Heat Detection
- Motor spinup

# Key Areas of the code


## robot launch.py is the launch file, it creates the package, brings up the required nodes

#### bringing up of nodes:

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),

- New nodes: 


        Node(
            package='turtlebot3_flywheels_and_Temperature',
            executable='flywheels_and_Temperature',
            output='screen'
        )

        Node(
            package='turtlebot3_navigation',
            executable='nav_controller',
            name='nav_controller',
            output='screen',
            parameters=[{
                'controller_frequency': 10.0,
            }],
            remappings=[('/cmd_vel', '/flywheel_cmd_vel')]
        )

#### bringing up of parameter files 

tb3_param_dir = LaunchConfiguration(
    'tb3_param_dir',
    default=os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'param',
        TURTLEBOT3_MODEL + '.yaml'))

#### bring up state publisher

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
    launch_arguments={'use_sim_time': use_sim_time}.items(),
)

- Publishing the robot's joint states
- Broadcasting the tf tree
