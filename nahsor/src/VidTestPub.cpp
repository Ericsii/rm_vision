#include "rm_nahsor/nahsor_node.hpp"
#include "tf2/LinearMath/Transform.h"

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(): Node("TestImage_publisher")
    {
        rclcpp::Node::SharedPtr node1 = std::make_shared<rclcpp::Node>("VidTest");
//创建Publisher
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr VidTestPub = node1->create_publisher<sensor_msgs::msg::Image>(
            "VidTestImg", 10);
//读取视频
        using namespace cv;
        VideoCapture capture;
        // capture.open("/home/star/data/lbr_fan.mp4");
        capture.open("/home/star/Desktop/ET.mp4");
        if (!capture.isOpened()) {
            RCLCPP_INFO(this->get_logger(),"could not read this video file...");
        }
//CvtoRos
        // cv::namedWindow("VidTest", cv::WINDOW_AUTOSIZE);
        while (rclcpp::ok) {
            // cv::imshow("VidTest",frame);
            // cv::waitKey(10);
            cv::Mat frame;
            capture.read(frame);
            // cv::imshow("origin", frame);
            // cv::waitKey(1);
            cv_bridge::CvImage cvi_rgb;
            cvi_rgb.header.stamp = this->get_clock()->now();
            cvi_rgb.header.frame_id = "VidTestImgOrigin";
            cvi_rgb.encoding = "bgr8";
            cvi_rgb.image = frame;
            sensor_msgs::msg::Image im_rgb;
            cvi_rgb.toImageMsg(im_rgb);
            // cv::imshow("view", cv_bridge::toCvShare(im_rgb,node1, "bgr8")->image);
            // cv::waitKey(5);
//Publish
            if(!frame.empty())
            VidTestPub->publish(im_rgb);
            cv::waitKey(20);
        }
    }
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
