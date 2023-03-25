#include <ros/ros.h>
#include <openhab_msgs/DateTimeCommand.h>

class DateTimePublisher
{
public:
    DateTimePublisher(const std::string& item_name)
        : item_name_(item_name),
          pub_(nh_.advertise<openhab_msgs::DateTimeCommand>("/openhab/items/" + item_name_ + "/command", 10)),
          rate_(10) // 10hz
    {
        if (enable_)
        {
            start();
        }
        else
        {
            stop();
        }
    }

    void start()
    {
        // Turn on publisher.
        enable_ = true;
        pub_ = nh_.advertise<openhab_msgs::DateTimeCommand>("/openhab/items/" + item_name_ + "/command", 10);

        while (ros::ok())
        {
            openhab_msgs::DateTimeCommand message;

            message.command = ros::Time::now();

            message.isnull = false;

            message.header.stamp = ros::Time::now();
            message.header.frame_id = "/base_link";
            message.item = item_name_;

            std::string message_str = "Publishing to " + message.item + " at " + std::to_string(ros::Time::now().toSec()) + ": command = " + std::to_string(message.command.toSec());
            ROS_INFO("%s", message_str.c_str());

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
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate rate_;
    bool enable_{false};
};

int main(int argc, char** argv)
{
    // Initialize the node and name it.
    ros::init(argc, argv, "DateTimePublisherNode", ros::init_options::AnonymousName);
    // Go to class functions that do all the heavy lifting.

    DateTimePublisher dateTimePublisher("testDateTime");

    try
    {
        dateTimePublisher.start();
    }
    catch (const ros::Exception& e)
    {
        ROS_ERROR("%s", e.what());
    }

    // Allow ROS to go to all callbacks.
    // spin() simply keeps the node from exiting until this node is stopped
    ros::spin();
    return 0;
}
