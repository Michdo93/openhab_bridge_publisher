#!/usr/bin/python
import rospy
from openhab_msgs.msg import LocationCommand

class LocationPublisher(object):
    """Node example class."""

    def __init__(self, item_name):

        self.item_name = item_name
        self.pub = rospy.Publisher("/openhab/items/" + self.item_name + "/command", LocationCommand, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz

        # Initialize message variables.
        self.enable = False
        self.message = ""

        if self.enable:
            self.start()
        else:
            self.stop()

    def start(self):
        """Turn on publisher."""
        self.enable = True
        self.pub = rospy.Publisher("/openhab/items/" + self.item_name + "/command", LocationCommand, queue_size=10)

        while not rospy.is_shutdown():

            self.message = LocationCommand()

            self.message.latitude = 48.0501442
            self.message.longitude = 8.2014192
            self.message.altitude = 857.0

            self.message.header.stamp = rospy.Time.now()
            self.message.header.frame_id = "/base_link"
            self.message.item = self.item_name

            message = "Publishing to %s at %s: latitude = %s, longitude = %s, altitude = %s" % (self.message.item, rospy.get_time(), self.message.latitude, self.message.longitude, self.message.altitude)
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
    rospy.init_node("LocationPublisherNode", anonymous=True)
    # Go to class functions that do all the heavy lifting.

    locationPublisher = LocationPublisher("testLocation")

    try:
        locationPublisher.start()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
