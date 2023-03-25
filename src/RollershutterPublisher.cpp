#include <ros/ros.h>
#include <openhab_msgs/RollershutterCommand.h>

class RollershutterPublisher {
  public:
    RollershutterPublisher(std::string item_name) {
        item_name_ = item_name;
        pub_ = nh_.advertise<openhab_msgs::RollershutterCommand>("/openhab/items/" + item_name_ + "/command", 10);
        rate_ = ros::Rate(10); // 10hz
    }

    void start() {
        enable_ = true;
        pub_ = nh_.advertise<openhab_msgs::RollershutterCommand>("/openhab/items/" + item_name_ + "/command", 10);

        while (ros::ok()) {
            message_.command = "DOWN";
            message_.iscommand = true;
            message_.ispercentage = false;
            message_.percentage = 0;
            message_.isnull = false;
            message_.header.stamp = ros::Time::now();
            message_.header.frame_id = "/base_link";
            message_.item = item_name_;

            std::stringstream ss;
            ss << "Publishing to " << message_.item << " at " << ros::Time::now() << ": command = " << message_.command << ", iscommand = " << message_.iscommand << ", ispercentage = " << message_.ispercentage << ", percentage = " << message_.percentage;
            ROS_INFO_STREAM(ss.str());

            pub_.publish(message_);
            rate_.sleep();
        }
    }

    void stop() {
        enable_ = false;
        pub_.shutdown();
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Rate rate_;
    std::string item_name_;
    bool enable_ = false;
    openhab_msgs::RollershutterCommand message_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RollershutterPublisherNode");
    RollershutterPublisher rollershutterPublisher("testRollershutter");

    try {
        rollershutterPublisher.start();
    } catch (const ros::Exception& e) {
        ROS_ERROR_STREAM("RollershutterPublisherNode exception: " << e.what());
    }

    ros::spin();
    return 0;
}
