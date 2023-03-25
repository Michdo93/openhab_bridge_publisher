#include "ros/ros.h"
#include "openhab_msgs/NumberCommand.h"

class NumberPublisher {
public:
  NumberPublisher(std::string item_name) : item_name_(item_name), enable_(false) {
    pub_ = nh_.advertise<openhab_msgs::NumberCommand>("/openhab/items/" + item_name_ + "/command", 10);
    rate_ = ros::Rate(10);
    if (enable_) {
      start();
    } else {
      stop();
    }
  }

  void start() {
    enable_ = true;
    pub_ = nh_.advertise<openhab_msgs::NumberCommand>("/openhab/items/" + item_name_ + "/command", 10);
    int i = 0;
    while (ros::ok()) {
      openhab_msgs::NumberCommand msg;
      i++;
      msg.command = static_cast<float>(i);
      msg.isnull = false;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "/base_link";
      msg.item = item_name_;
      std::string message = "Publishing to " + msg.item + " at " + std::to_string(ros::Time::now().toSec()) +
                            ": command = " + std::to_string(msg.command);
      ROS_INFO("%s", message.c_str());
      pub_.publish(msg);
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
  bool enable_;
};

int main(int argc, char **argv) {
  // Initialize the node and name it.
  ros::init(argc, argv, "NumberPublisherNode");
  NumberPublisher numberPublisher("testNumber");
  try {
    numberPublisher.start();
  } catch (const ros::Exception &e) {
    // Do nothing
  }
  // spin() simply keeps the node running until it's stopped
  ros::spin();
  return 0;
}
