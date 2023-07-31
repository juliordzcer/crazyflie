### Trajectory

The desired trajectories are stored in `trajectory_node.py` and `trajectory_follower_node.py`.

### Joystick
Joystick configuration is stored in `joystick.py`

### Attitude Controller
The operation of the `select_controller.py` code is to change the attitude controllers, without having to change the firmware.

### Leader information for multi-agents
In order to send the necessary information for multi-agent controllers, from the leader to the followers, in the `publish_info_leader.py` script, create a topic to send the leader's acceleration.
