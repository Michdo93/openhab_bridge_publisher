#include <ros/ros.h>
#include <openhab_msgs/ColorCommand.h>

class ColorPublisher
{
public:
    ColorPublisher(std::string item_name)
    {
        item_name_ = item_name;
        pub_ = nh_.advertise<openhab_msgs::ColorCommand>("/openhab/items/" + item_name_ + "/command", 10);
        rate_ = ros::Rate(10); // 10Hz
        enable_ = false;
        message_ = NULL;

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
        enable_ = true;
        pub_ = nh_.advertise<openhab_msgs::ColorCommand>("/openhab/items/" + item_name_ + "/command", 10);

        while (ros::ok())
        {
            message_.command = "OFF";
            message_.iscommand = false;
            message_.ispercentage = false;
            message_.percentage = 0;
            message_.ishsb = true;
            message_.hue = 127;
            message_.saturation = 61;
            message_.brightness = 99;
            message_.isnull = false;

            message_.header.stamp = ros::Time::now();
            message_.header.frame_id = "/base_link";
            message_.item = item_name_;

            std::string message = "Publishing to " + message_.item + " at " + std::to_string(ros::Time::now().toSec()) + ": command = " + message_.command + ", iscommand = " + std::to_string(message_.iscommand) + ", ispercentage = " + std::to_string(message_.ispercentage) + ", percentage = " + std::to_string(message_.percentage) + ", ishsb = " + std::to_string(message_.ishsb) + ", hue = " + std::to_string(message_.hue) + ", saturation = " + std::to_string(message_.saturation) + ", brightness = " + std::to_string(message_.brightness);
            ROS_INFO("%s", message.c_str());

            pub_.publish(message_);
            rate_.sleep();
        }
    }

    void stop()
    {
        enable_ = false;
        pub_.shutdown();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate rate_;
    bool enable_;
    std::string item_name_;
    openhab_msgs::ColorCommand message_;
};

int main(int argc, char **argv)
{
    // Initialize the node and name it.
    ros::init(argc, argv, "ColorPublisherNode");

    // Create an object of class ColorPublisher that will take care of everything.
    ColorPublisher colorPublisher("testColor");

    try
    {
        colorPublisher.start();
    }
    catch (ros::Exception &e)
    {
        ROS_ERROR("Error occurred: %s", e.what());
    }

    // Allow ROS to go to all callbacks.
    ros::spin();
    
    return 0;
}
