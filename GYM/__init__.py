#import logging
from gym.envs.registration import register

#logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

# envs

# Quad envs
register(
    id='GazeboQuadEnv-v0',
    entry_point='GYM.env:GazeboQuadEnv',
)
