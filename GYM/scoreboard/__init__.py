from gym.scoreboard.registration import registry, add_task, add_group

add_group(
    id='gazebo',
    name='Gazebo',
    description='TODO.'
)

add_task(
    id = 'GazeboQuadEnv-v0',
    group = 'gazebo',
    summary = 'hover',
)

registry.finalize()
