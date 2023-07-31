#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from crazyflie.srv import UpdateParams

class PublishExternalPosition:
    def __init__(self):
        rospy.init_node('publish_external_position_vrpn', anonymous=True)
        self.topic = rospy.get_param("~topic", "/vrpn_client_node/crazyflie1/pose")
        rospy.wait_for_service('update_params')
        rospy.loginfo("Found update_params service")
        self.update_params = rospy.ServiceProxy('update_params', UpdateParams)

        self.first_transform = True

        self.pub = rospy.Publisher("external_pose", PoseStamped, queue_size=1)
        self.msg = PoseStamped()

        rospy.Subscriber(self.topic, PoseStamped, self.on_new_transform)

    def on_new_transform(self, pose):
        if self.first_transform:
            # Initialize kalman filter
            rospy.set_param("kalman/initialX", pose.pose.position.x)
            rospy.set_param("kalman/initialY", pose.pose.position.y)
            rospy.set_param("kalman/initialZ", pose.pose.position.z)

            # Update multiple parameters using the 'update' method
            self.update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ",
                                "kalman/resetEstimation"])

            self.first_transform = False

        # Update the pose message with the new values
        self.msg.header = pose.header
        self.msg.pose = pose.pose
        self.pub.publish(self.msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.sleep(12)
    external_position = PublishExternalPosition()
    external_position.run()
