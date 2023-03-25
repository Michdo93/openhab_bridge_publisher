#include <ros/ros.h>
#include <os>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <openhab_msgs/ImageCommand.h>

using namespace cv;

class ImagePublisher
{
    public:
        ImagePublisher(std::string item_name, std::string image_path) : 
            item_name_(item_name),
            image_path_(image_path),
            enable_(false)
        {
            pub_ = nh_.advertise<openhab_msgs::ImageCommand>("/openhab/items/" + item_name + "/command", 10);
            rate_ = ros::Rate(10);
            bridge_ = cv_bridge::CvBridge();

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
            pub_ = nh_.advertise<openhab_msgs::ImageCommand>("/openhab/items/" + item_name_ + "/command", 10);

            while (ros::ok())
            {
                openhab_msgs::ImageCommand message;

                if (std::filesystem::exists(image_path_))
                {
                    message.isnull = false;
                    Mat img = imread(image_path_);
                }
                else
                {
                    message.isnull = true;
                    image_path_ = "NULL";
                    Mat img = Mat::zeros(Size(100, 100), CV_8UC3);
                }

                // finally convert RGB image to BGR for opencv
                cvtColor(img, img, COLOR_RGB2BGR);

                try
                {
                    sensor_msgs::ImagePtr image_msg = bridge_.cv2_to_imgmsg(img, "bgr8");
                    message.command = *image_msg;
                    message.header.stamp = ros::Time::now();
                    message.header.frame_id = "/base_link";
                    message.item = item_name_;

                    std::string log_message = "Publishing " + image_path_ + " to " + message.item + " at " + std::to_string(ros::Time::now().toSec());
                    ROS_INFO("%s", log_message.c_str());

                    pub_.publish(message);
                }
                catch (cv_bridge::CvBridgeError& e)
                {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                }

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
        cv_bridge::CvBridge bridge_;
        std::string item_name_;
        std::string image_path_;
        bool enable_;
};

int main(int argc, char** argv)
{
    // Initialize the node and name it.
    ros::init(argc, argv, "ImagePublisherNode");
    ros::NodeHandle nh;

    // Go to class functions that do all the heavy lifting.
    std::string image_path;
    nh.getParam("image", image_path);

    if (image_path.empty())
    {
        image_path = "Image.jpg";
    }

    ImagePublisher imagePublisher("testImage", image_path);

    try
    {
        imagePublisher.start();
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("An exception occurred: %s", e.what());
    }

    ros::spin();

    return 0;
}
