#include <ros/ros.h>
#include <openhab_msgs/ContactCommand.h>

class ContactPublisher
{
private:
    std::string item_name;
    ros::Publisher pub;
    ros::Rate rate;

    // Initialize message variables.
    bool enable;
    openhab_msgs::ContactCommand message;

public:
    ContactPublisher(std::string item_name)
        : item_name(item_name)
        , pub(ros::NodeHandle().advertise<openhab_msgs::ContactCommand>("/openhab/items/" + item_name + "/command", 10))
        , rate(10)
        , enable(false)
    {}

    void start()
    {
        // Turn on publisher.
        enable = true;
        pub = ros::NodeHandle().advertise<openhab_msgs::ContactCommand>("/openhab/items/" + item_name + "/command", 10);

        while (ros::ok())
        {
            message.command = openhab_msgs::ContactCommand::OPEN;
            message.isnull = false;

            message.header.stamp = ros::Time::now();
            message.header.frame_id = "/base_link";
            message.item = item_name;

            std::string log_message = "Publishing to " + message.item + " at " + std::to_string(ros::Time::now().toSec()) + ": command = " + std::to_string(message.command);
            ROS_INFO("%s", log_message.c_str());

            pub.publish(message);
            rate.sleep();
        }
    }

    void stop()
    {
        // Turn off publisher.
        enable = false;
        pub.shutdown();
    }
};

int main(int argc, char **argv)
{
    // Initialize the node and name it.
    ros::init(argc, argv, "ContactPublisherNode");
    // Go to class functions that do all the heavy lifting.

    ContactPublisher contactPublisher("testContact");

    try
    {
        contactPublisher.start();
    }
    catch (ros::Exception &e)
    {
        // Do nothing.
    }

    // Allow ROS to go to all callbacks.
    // spin() simply keeps C++ from exiting until this node is stopped
    ros::spin();

    return 0;
}
