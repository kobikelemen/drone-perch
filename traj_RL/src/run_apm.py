#!/usr/bin/env python
import roslaunch



uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/kobi/catkin_ws/src/iq_sim/launch/apm.launch"])
launch.start()
