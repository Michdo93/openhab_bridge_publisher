#include <ros/ros.h>
#include "openhab_msgs/DimmerCommand.h"

class DimmerPublisher
{
public:
    // Constructor
    DimmerPublisher(const std::string& item_name)
        : item_name_(item_name), enable_(false)
    {
        pub_ = nh_.advertise<openhab_msgs::DimmerCommand>("/openhab/items/" + item_name_ + "/command", 10);
        rate_ = ros::Rate(10); // 10hz
        stop();
    }

    void start()
    {
        // Turn on publisher.
        enable_ = true;
        pub_ = nh_.advertise<openhab_msgs::DimmerCommand>("/openhab/items/" + item_name_ + "/command", 10);

        while (ros::ok() && enable_)
        {
            openhab_msgs::DimmerCommand message;

            message.command = openhab_msgs::DimmerCommand::ON;
            message.iscommand = false;
            message.ispercentage = true;
            message.percentage = openhab_msgs::DimmerCommand::PERCENTAGE80;

            message.isnull = false;

            message.header.stamp = ros::Time::now();
            message.header.frame_id = "/base_link";
            message.item = item_name_;

            std::stringstream ss;
            ss << "Publishing to " << message.item << " at " << ros::Time::now() << ": command = " << message.command
               << ", iscommand = " << message.iscommand << ", ispercentage = " << message.ispercentage
               << ", percentage = " << message.percentage;
            ROS_INFO_STREAM(ss.str());

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
    std::string item_name_;
    bool enable_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate rate_;
};

int main(int argc, char** argv)
{
    // Initialize the node and name it.
    ros::init(argc, argv, "DimmerPublisherNode", ros::init_options::AnonymousName);

    // Go to class functions that do all the heavy lifting.
    DimmerPublisher dimmer_publisher("testDimmer");

    try
    {
        dimmer_publisher.start();
    }
    catch (const ros::Exception& e)
    {
        ROS_ERROR_STREAM("ROS Exception: " << e.what());
    }

    // Allow ROS to go to all callbacks.
    // spin() simply keeps C++ from exiting until this node is stopped.
    ros::spin();

    return 0;
}
