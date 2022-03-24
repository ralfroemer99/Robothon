#!/home/pierre/envs/general/bin/python

import rospy
from franka_msgs.msg import FrankaState
from sys import getsizeof, exit

class Detector:
    def __init__(self):
        self.state_buffer  = []
        self.subscriber = rospy.Subscriber("franka_state_controller/franka_states", FrankaState, self.callback)
        rospy.init_node("self_collision_detector", anonymous = False)
        self.rate = rospy.Rate(28)
        rospy.spin()

    def callback(self, msg):
        self.state_buffer.append(msg)
        print(msg)
        exit(0)
        #print(msg.__sizeof__())

        # Pop the oldest franka state message
        if len(self.state_buffer) > 50:
            self.state_buffer = self.state_buffer[1:]
        #print(len(self.state_buffer))


if __name__ == "__main__":
    detector = Detector()