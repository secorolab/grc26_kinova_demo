from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()

    arm1     = 'kinova1'
    gripper1 = 'g1'

    arm2     = 'kinova2'
    gripper2 = 'g2'

    # arm 1 client
    sl.node(
        package='moveit_client',
        executable='client_node',
        name=arm1 + '_client',
        parameters=[{
            'arm': arm1,
            'gripper': gripper1
        }]
    )

    # arm 2 client
    sl.node(
        package='moveit_client',
        executable='client_node',
        name=arm2 + '_client',
        parameters=[{
            'arm': arm2,
            'gripper': gripper2
        }]
    )

    # pick and place node
    sl.node(
        package='grc26_isaacsim_task',
        executable='dualarm_pick_place',
        name='dualarm_pick_place',
        parameters=[{
            'arm1': arm1,
            'gripper1': gripper1,
            'arm2': arm2,
            'gripper2': gripper2
        }]
    )

    return sl.launch_description()
