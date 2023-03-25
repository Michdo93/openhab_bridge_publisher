#include <ros/ros.h>
#include <openhab_msgs/SwitchCommand.h>

class SwitchPublisher
{
public:
    SwitchPublisher(std::string item_name)
    {
        item_name_ = item_name;
        pub_ = nh_.advertise<openhab_msgs::SwitchCommand>("/openhab/items/" + item_name_ + "/command", 10);
        rate_ = ros::Rate(10); // 10hz
        enable_ = false;
    }

    void start()
    {
        // Turn on publisher.
        enable_ = true;
        pub_ = nh_.advertise<openhab_msgs::SwitchCommand>("/openhab/items/" + item_name_ + "/command", 10);

        while (ros::ok())
        {
            openhab_msgs::SwitchCommand message;
            message.command = "ON";
            message.isnull = false;
            message.header.stamp = ros::Time::now();
            message.header.frame_id = "/base_link";
            message.item = item_name_;

            std::string log_message = "Publishing to " + message.item + " at " + std::to_string(ros::Time::now().toSec()) + ": command = " + message.command;
            ROS_INFO_STREAM(log_message);

            pub_.publish(message);
            rate_.sleep();
        }
    }

    void stop()
    {
        // Turn off publisher.
        enable_ = false;
        pub_.shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate rate_;
    std::string item_name_;
    bool enable_;
};

int main(int argc, char **argv)
{
    // Initialize the node and name it.
    ros::init(argc, argv, "SwitchPublisherNode");
    // Go to class functions that do all the heavy lifting.
    SwitchPublisher switch_publisher("testSwitch");

    try
    {
        switch_publisher.start();
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR_STREAM("ROS Exception: " << e.what());
    }

    // Allow ROS to go to all callbacks.
    // spin() simply keeps c++ from exiting until this node is stopped
    ros::spin();

    return 0;
}
