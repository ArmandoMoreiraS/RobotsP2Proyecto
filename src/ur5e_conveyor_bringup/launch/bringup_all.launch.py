from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('ur5e_conveyor_bringup')
    world = os.path.join(pkg, 'worlds', 'conveyor_with_attacher.world')
    red  = os.path.join(pkg, 'urdf',   'red_box.sdf')
    blue = os.path.join(pkg, 'urdf',   'blue_box.sdf')

    ur_x = DeclareLaunchArgument('ur_x', default_value='-0.6')
    ur_y = DeclareLaunchArgument('ur_y', default_value='0.3')
    ur_z = DeclareLaunchArgument('ur_z', default_value='0.85')
    red_x = DeclareLaunchArgument('red_x', default_value='0.0')
    red_y = DeclareLaunchArgument('red_y', default_value='0.0')
    red_z = DeclareLaunchArgument('red_z', default_value='0.90')
    blue_x = DeclareLaunchArgument('blue_x', default_value='0.15')
    blue_y = DeclareLaunchArgument('blue_y', default_value='0.0')
    blue_z = DeclareLaunchArgument('blue_z', default_value='0.90')

    # MODELOS: .../install/conveyorbelt_gazebo/share/conveyorbelt_gazebo/models
    model_path = os.path.join(get_package_share_directory('conveyorbelt_gazebo'), 'models')

    # PLUGINS: .../install/ros2_linkattacher/lib
    share_attacher = get_package_share_directory('ros2_linkattacher')
    install_prefix = os.path.dirname(os.path.dirname(share_attacher))
    plugin_path = os.path.join(install_prefix, 'lib')

    set_model_env  = SetEnvironmentVariable('GAZEBO_MODEL_PATH',  model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH',''))
    set_plugin_env = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path + ':' + os.environ.get('GAZEBO_PLUGIN_PATH',''))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gen_urdf = ExecuteProcess(
        cmd=['bash','-lc',
             'ros2 run xacro xacro --inorder '
             '$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro '
             'ur_type:=ur5e prefix:="" name:=ur5e joint_limited:=true safety_limits:=true '
             'safety_pos_margin:=0.15 safety_k_position:=20 > /tmp/ur5e.urdf'],
        shell=False
    )
    spawn_ur = ExecuteProcess(
        cmd=['ros2','run','gazebo_ros','spawn_entity.py',
             '-entity','ur5e','-file','/tmp/ur5e.urdf',
             '-x', LaunchConfiguration('ur_x'),
             '-y', LaunchConfiguration('ur_y'),
             '-z', LaunchConfiguration('ur_z')],
        shell=False
    )
    spawn_red = ExecuteProcess(
        cmd=['ros2','run','gazebo_ros','spawn_entity.py',
             '-entity','red_box_1','-file', red,
             '-x', LaunchConfiguration('red_x'),
             '-y', LaunchConfiguration('red_y'),
             '-z', LaunchConfiguration('red_z')],
        shell=False
    )
    spawn_blue = ExecuteProcess(
        cmd=['ros2','run','gazebo_ros','spawn_entity.py',
             '-entity','blue_box_1','-file', blue,
             '-x', LaunchConfiguration('blue_x'),
             '-y', LaunchConfiguration('blue_y'),
             '-z', LaunchConfiguration('blue_z')],
        shell=False
    )

    color_node = ExecuteProcess(cmd=['ros2','run','color_sorting_perception','color_sorter'], shell=False)
    palletizer  = ExecuteProcess(cmd=['ros2','run','palletizer_move','palletizer'], shell=False)

    return LaunchDescription([
        ur_x, ur_y, ur_z, red_x, red_y, red_z, blue_x, blue_y, blue_z,
        set_model_env, set_plugin_env,
        gazebo,
        TimerAction(period=1.5, actions=[gen_urdf]),
        TimerAction(period=2.5, actions=[spawn_ur]),
        TimerAction(period=3.0, actions=[spawn_red]),
        TimerAction(period=3.2, actions=[spawn_blue]),
        TimerAction(period=3.5, actions=[color_node]),
        TimerAction(period=3.7, actions=[palletizer]),
    ])
