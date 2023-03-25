#include "ros/ros.h"
#include "openhab_msgs/LocationCommand.h"

class LocationPublisher {
public:
    LocationPublisher(std::string item_name)
        : item_name_(item_name),
          pub_(nh_.advertise<openhab_msgs::LocationCommand>("/openhab/items/" + item_name_ + "/command", 10)),
          rate_(10)
    {
        // Initialize message variables.
        enable_ = false;
        message_ = openhab_msgs::LocationCommand();
    }

    void start() {
        // Turn on publisher.
        enable_ = true;
        pub_ = nh_.advertise<openhab_msgs::LocationCommand>("/openhab/items/" + item_name_ + "/command", 10);

        while (ros::ok()) {
            message_.latitude = 48.0501442;
            message_.longitude = 8.2014192;
            message_.altitude = 857.0;
            message_.isnull = false;
            message_.header.stamp = ros::Time::now();
            message_.header.frame_id = "/base_link";
            message_.item = item_name_;

            std::string message = "Publishing to " + message_.item + " at " + std::to_string(ros::Time::now().toSec()) + ": latitude = " + std::to_string(message_.latitude) + ", longitude = " + std::to_string(message_.longitude) + ", altitude = " + std::to_string(message_.altitude);
            ROS_INFO("%s", message.c_str());

            pub_.publish(message_);
            rate_.sleep();
        }
    }

    void stop() {
        // Turn off publisher.
        enable_ = false;
        pub_.shutdown();
    }

private:
    std::string item_name_;
    bool enable_;
    openhab_msgs::LocationCommand message_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate rate_;
};

int main(int argc, char **argv) {
    // Initialize the node and name it.
    ros::init(argc, argv, "LocationPublisherNode");
    // Go to class functions that do all the heavy lifting.
    LocationPublisher location_publisher("testLocation");
    try {
        location_publisher.start();
    } catch (const ros::Exception &e) {
        ROS_ERROR("%s", e.what());
    }
    // Allow ROS to go to all callbacks.
    // spin() simply keeps C++ from exiting until this node is stopped.
    ros::spin();
    return 0;
}
