#!/usr/bin/python
import rospy
from openhab_msgs.msg import ContactCommand

class ContactPublisher(object):
    """Node example class."""

    def __init__(self, item_name):

        self.item_name = item_name
        self.pub = rospy.Publisher("/openhab/items/" + self.item_name + "/command", ContactCommand, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # Initialize message variables.
        self.enable = False
        self.message = None

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher."""
        self.enable = True
        self.pub = rospy.Publisher("/openhab/items/" + self.item_name + "/command", ContactCommand, queue_size=10)

        while not rospy.is_shutdown():

            self.message = ContactCommand()

            self.message.command = ContactCommand.OPEN
            
            self.message.isnull = False

            self.message.header.stamp = rospy.Time.now()
            self.message.header.frame_id = "/base_link"
            self.message.item = self.item_name

            message = "Publishing to %s at %s: command = %s" % (self.message.item, rospy.get_time(), self.message.command)
            rospy.loginfo(message)

            self.pub.publish(self.message)
            self.rate.sleep()

    def stop(self):
        """Turn off publisher."""
        self.enable = False
        self.pub.unregister()

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("ContactPublisherNode", anonymous=True)
    # Go to class functions that do all the heavy lifting.

    contactPublisher = ContactPublisher("testContact")

    try:
        contactPublisher.start()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
