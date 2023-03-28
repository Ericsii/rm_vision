/*
 * @Author: your name
 * @Date: 2021-11-17 11:43:44
 * @LastEditTime: 2021-12-01 22:37:13
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /scu_rm_ros/src/nahsor_modules/include/nahsor_node.hpp
 */
#ifndef NAHSOR_NODE_HPP_
#define NAHSOR_NODE_HPP_
#define RM_DEBUG_MODE
#include <memory>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>

#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "rm_util/coordinate_tool.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nahsor
{
    enum class TaskMode
    {
        idle,
        small,
        large,
        test
    };

    class NahsorNode
    {
    public:
        explicit NahsorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

    private:
        /**
         * @brief 主流程函数，获取装甲板预测位置
         *
         * @param img 相机获取的图像
         * @param time_stamp 时间戳
         */
        void process_image(std_msgs::msg::Header header, cv::Mat &img, geometry_msgs::msg::Quaternion q);
        /**
         * @brief 更改目标模式的回调函数
         *
         * @param request
         * @param response
         * @return true
         * @return false
         */
        void process_image_test(sensor_msgs::msg::Image imgt);
        void new_process_image_test(sensor_msgs::msg::Image imgt);

        bool setModeCallBack(
            const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
            std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

        bool set_color_cb(
            const std::shared_ptr<rm_interfaces::srv::SetColor::Request> request,
            std::shared_ptr<rm_interfaces::srv::SetColor::Response> response);
        void gimbalStateCallback(const rm_interfaces::msg::Gimbal::SharedPtr msg);
        void taskImageProcess(cv::Mat &img, double img_stamp);

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<rm_cam::CamClient> cam_client_;
        std::shared_ptr<rm_cam::WrapperClient> wrapper_client_;

//视频测试
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr VidTestPub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr VidTestSub;

        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_ctrl_pub_;
        rclcpp::Subscription<rm_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
        rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

        std::shared_ptr<NahsorAlgo> nahsor_algo;
        std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool;
        std::shared_ptr<rm_trajectory::GravitySolver> gravity_solver;
        std::shared_ptr<rm_trajectory::TransformTool> trajectory_transform_tool;
        TaskMode current_mode_{TaskMode::small};

        Eigen::Quaterniond curr_pose_;
        u_int8_t gimbal_cmd_id{0};

        std::vector<cv::Point3f> mBigArmorPoints;
        Eigen::Quaterniond cam2imu_static_;

        double shoot_delay = 0.55;
        double yaw_offset = -1.;
        double pitch_offset = 1.;
    };
}

#endif