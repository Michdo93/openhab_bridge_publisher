#include <ros/ros.h>
#include <openhab_msgs/StringCommand.h>

class StringPublisher {
  public:
    StringPublisher(std::string item_name) : item_name(item_name), rate(10), enable(false) {
      pub = n.advertise<openhab_msgs::StringCommand>("/openhab/items/" + item_name + "/command", 10);
    }

    void start() {
      enable = true;
      pub = n.advertise<openhab_msgs::StringCommand>("/openhab/items/" + item_name + "/command", 10);

      while (ros::ok()) {
        openhab_msgs::StringCommand message;

        message.command = "Hello World " + std::to_string(ros::Time::now().toSec());
        message.isnull = false;

        message.header.stamp = ros::Time::now();
        message.header.frame_id = "/base_link";
        message.item = item_name;

        ROS_INFO_STREAM("Publishing to " << message.item << " at " << ros::Time::now() << ": command = " << message.command);

        pub.publish(message);
        rate.sleep();
      }
    }

    void stop() {
      enable = false;
      pub.shutdown();
    }

  private:
    std::string item_name;
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Rate rate;
    bool enable;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "StringPublisherNode", ros::init_options::AnonymousName);
  StringPublisher stringPublisher("testString");
  try {
    stringPublisher.start();
  } catch (ros::Exception& e) {
    // do nothing
  }
  ros::spin();
  return 0;
}
