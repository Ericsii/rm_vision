#include "rm_nahsor/nahsor_node.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


namespace nahsor
{
    NahsorNode::NahsorNode(const rclcpp::NodeOptions &options)
    {
        node_ = std::make_shared<rclcpp::Node>("Nahsor_aim", options);
        // VidTestSub = node_->create_subscription<sensor_msgs::msg::Image>(
        // "VidTestImg", 1, std::bind(&NahsorNode::process_image_test, this, std::placeholders::_1));
        VidTestSub = node_->create_subscription<sensor_msgs::msg::Image>(
        "VidTestImg", 10, std::bind(&NahsorNode::new_process_image_test, this, std::placeholders::_1));
        std::string camera_name = "mv_camera";
        std::string robot_color = "red";
        std::string imu_name = "imu";
        bool auto_start = false;
        bool debug = true;

        //获取参数
        node_->declare_parameter("debug", debug);
        node_->declare_parameter("camera_name", camera_name);
        node_->declare_parameter("auto_start", auto_start);
        node_->declare_parameter("imu_name", imu_name);
        node_->declare_parameter("shoot_delay", shoot_delay);
        node_->declare_parameter("yaw_offset", yaw_offset);
        node_->declare_parameter("pitch_offset", pitch_offset);
        node_->get_parameter("debug", debug);
        node_->get_parameter("camera_name", camera_name);
        node_->get_parameter("auto_start", auto_start);
        node_->get_parameter("imu_name", imu_name);
        node_->get_parameter("shoot_delay", shoot_delay);
        node_->get_parameter("yaw_offset", yaw_offset);
        node_->get_parameter("pitch_offset", pitch_offset);

        float realHeight = 6.3;
        float realWidth = 22.2;
        float half_x = realWidth / 2;
        float half_y = realHeight / 2;
        mBigArmorPoints.emplace_back(-half_x, half_y, 0.);
        mBigArmorPoints.emplace_back(-half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, half_y, 0);

        Eigen::Matrix3d rotation;
        rotation << 1, 0, 0, 0, 0, 1, 0, -1, 0;
        cam2imu_static_ = Eigen::Quaterniond(rotation);

        using namespace std::placeholders;
        gimbal_cmd_pub_ = node_->create_publisher<rm_interfaces::msg::GimbalCmd>(
            "cmd_gimbal", 10);
        set_mode_srv_ = node_->create_service<rm_interfaces::srv::SetMode>(
            "nahsor/set_mode", std::bind(&NahsorNode::setModeCallBack, this, _1, _2));

#ifndef RM_DEBUG_MODE
        // wrapper_client_ = std::make_shared<rm_cam::WrapperClient>(
        //     node_, camera_name, imu_name, std::bind(&NahsorNode::process_image, this, _1, _2, _3));
#endif

        point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
            "world/nahsor_point", 10);

        RCLCPP_INFO(node_->get_logger(), "Wrapper client create success.");
        sensor_msgs::msg::CameraInfo info;

#ifndef RM_DEBUG_MODE
        if (!wrapper_client_->get_camera_info(info))
        {
            RCLCPP_ERROR(node_->get_logger(), "get camera info failed!");
            return;
        }
#endif
        std::ostringstream oss;
        oss << "k:";
        for (auto &x : info.k)
        {
            oss << x << " ";
        }
        oss << ",d:";
        for (auto &x : info.d)
        {
            oss << x << " ";
        }
        RCLCPP_INFO(node_->get_logger(), "get camera info: %s", oss.str().c_str());
        std::vector<double> camera_k(9, 0);
        std::copy_n(info.k.begin(), 9, camera_k.begin());

        // mono_location_tool = std::make_shared<rm_util::MonoMeasureTool>(camera_k, info.d);

        nahsor_algo = std::make_shared<NahsorAlgo>(node_, mono_location_tool);
        nahsor_algo->set_delay(shoot_delay);
        gravity_solver = std::make_shared<rm_trajectory::GravitySolver>(30, 0.03);
        trajectory_transform_tool = std::make_shared<rm_trajectory::TransformTool>(gravity_solver);
        if (auto_start)
        {
            current_mode_ = TaskMode::large;
            // wrapper_client_->start();
            // RCLCPP_INFO(node_->get_logger(), "auto start!");
        }
    }

#ifdef RM_DEBUG_MODE


    void NahsorNode::new_process_image_test(sensor_msgs::msg::Image imgt)
    {

        cv::Mat img;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(imgt, "bgr8");
        cv_ptr->image.copyTo(img);
        cv::Mat temp = img.clone();

        int res = -1;
        double time_stamp_s = imgt.header.stamp.sec + imgt.header.stamp.nanosec * 1e-9;
        // res = nahsor_algo->bigPred_test(img,time_stamp_s);
        res = nahsor_algo->bigPred_new(img,time_stamp_s);

        if (res == 0)
        {
            auto point = nahsor_algo->get_target();
            auto R_point = nahsor_algo->get_R_center();
            // RCLCPP_INFO(node_->get_logger(), "target_x:%f,targte_y:%f",point.x,point.y);
            // RCLCPP_ERROR(node_->get_logger(),"point_x:%f,point_y:%f,R_point_x:%f,R_point_y:%f",point.x,point.y,R_point.x,R_point.y);
            cv::resize(temp,temp,cv::Size(640,480));
            cv::circle(temp, point, 10, cv::Scalar(0, 255, 0));
            cv::circle(temp, R_point, 10, cv::Scalar(255, 0, 0));

            cv::imshow("VidTest", temp);
            cv::waitKey(1);

            // rclcpp::sleep_for(std::chrono::seconds(20));

            // float pitch, yaw;
            
            // mono_location_tool->calc_view_angle(point, pitch, yaw);
            // yaw = rm_util::rad_to_deg(yaw);

            // // PNP解算
            // double target_pitch, target_yaw;
            // cv::Point3f position;
            // cv::Mat rotation;
            // std::vector<cv::Point2f> points;
            // points.push_back(nahsor_algo->get_points()[0]);
            // points.push_back(nahsor_algo->get_points()[1]);
            // points.push_back(nahsor_algo->get_points()[2]);
            // points.push_back(nahsor_algo->get_points()[3]);
            // mono_location_tool->solve_pnp(points, mBigArmorPoints,
            //                               position, rotation);
            // Eigen::Vector3d position3d_camera(position.x, position.y, position.z);
            // Eigen::Vector3d position3d_world = curr_pose_ * cam2imu_static_ * position3d_camera;

            // // pitch弹道解算
            // trajectory_transform_tool->solve(position3d_world, target_pitch, target_yaw);
            // double target_distance = position3d_world.norm(); //水平距离
            // target_yaw = rm_util::rad_to_deg(atan2(position3d_world(0, 0), position3d_world(1, 0)));

            // rm_interfaces::msg::GimbalCmd gimbal_cmd;
            // gimbal_cmd.id = gimbal_cmd_id++;
            // gimbal_cmd.position.pitch = target_pitch + pitch_offset;
            // gimbal_cmd.position.yaw = -(target_yaw + yaw_offset);
            // gimbal_cmd_pub_->publish(gimbal_cmd);

            // geometry_msgs::msg::PointStamped target_point;
            // target_point.point.x = position3d_world(0) / 100;
            // target_point.point.y = position3d_world(1) / 100;
            // target_point.point.z = position3d_world(2) / 100;
            // target_point.header.frame_id = "imu_link";
            // point_pub_->publish(target_point);
            // cv::imshow("VidTest", temp);
            // cv::waitKey(1);

        }
        else{
            RCLCPP_WARN(node_->get_logger(), "Prediction Faild");
        }
        // else
        // {
        //     rm_interfaces::msg::GimbalCmd gimbal_cmd;
        //     gimbal_cmd.id = gimbal_cmd_id++;
        //     // gimbal_cmd.position.pitch = c_pitch;
        //     // gimbal_cmd.position.yaw = c_yaw;
        //     gimbal_cmd_pub_->publish(gimbal_cmd);
        // }
    }
#endif

//     void NahsorNode::process_image_test(sensor_msgs::msg::Image imgt)
//     {
//         // try
//         // {
//         //     cv::imshow("VidTest", cv_bridge::toCvShare(imgt,node_, "bgr8")->image);
//         //     cv::waitKey(10);
//         // }
//         // catch (cv_bridge::Exception& e)
//         // {
//         //     RCLCPP_INFO(node_->get_logger(),"Could not convert from '%s' to 'bgr8'.", imgt.encoding.c_str());
//         // }
//         cv::Mat img;
//         cv_bridge::CvImagePtr cv_ptr;
//         cv_ptr = cv_bridge::toCvCopy(imgt, "bgr8");
//         cv_ptr->image.copyTo(img);
//         cv::Mat temp = img.clone();
//         cv::imshow("VidTest",img);
//         cv::waitKey(10);
//         int res = -1;
//         // curr_pose_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
//         // auto euler_angles = rm_util::CoordinateTranslation::quat2euler(curr_pose_);
//         // float c_pitch, c_yaw, c_roll;
//         // c_pitch = rm_util::rad_to_deg(euler_angles(0));
//         // c_yaw = rm_util::rad_to_deg(euler_angles(2));
//         // c_roll = rm_util::rad_to_deg(euler_angles(1));
//         // (void)c_roll;
//         double time_stamp_s = imgt.header.stamp.sec + imgt.header.stamp.nanosec * 1e-9;
//         res = nahsor_algo->bigPred(img,time_stamp_s);

//         // RCLCPP_ERROR(node_->get_logger(), "res:%d",res);
//         // if (current_mode_ == TaskMode::small)
//         // {
//         //     res = nahsor_algo->smallPred(img);
//         // }
//         // if (current_mode_ == TaskMode::large)
//         // {
//         //     double time_stamp_s = header.stamp.sec + header.stamp.nanosec * 1e-9;
//         //     res = nahsor_algo->bigPred(img, time_stamp_s);
//         // }
//         // if (current_mode_ == TaskMode::test)
//         // {
//         //     res = nahsor_algo->noPred(img);
//         // }

//         if (res == 0)
//         {
//             auto point = nahsor_algo->get_target();
//             auto R_point = nahsor_algo->get_R_center();

//             // RCLCPP_ERROR(node_->get_logger(),"point_x:%f,point_y:%f,R_point_x:%f,R_point_y:%f",point.x,point.y,R_point.x,R_point.y);
//             cv::circle(temp, point, 10, cv::Scalar(0, 255, 0));
//             cv::circle(temp, R_point, 10, cv::Scalar(255, 0, 0));
//             cv::imshow("VidTest", temp);
//             cv::waitKey(3);

//             // rclcpp::sleep_for(std::chrono::seconds(20));

//             // float pitch, yaw;
            
//             // mono_location_tool->calc_view_angle(point, pitch, yaw);
//             // yaw = rm_util::rad_to_deg(yaw);

//             // // PNP解算
//             // double target_pitch, target_yaw;
//             // cv::Point3f position;
//             // cv::Mat rotation;
//             // std::vector<cv::Point2f> points;
//             // points.push_back(nahsor_algo->get_points()[0]);
//             // points.push_back(nahsor_algo->get_points()[1]);
//             // points.push_back(nahsor_algo->get_points()[2]);
//             // points.push_back(nahsor_algo->get_points()[3]);
//             // mono_location_tool->solve_pnp(points, mBigArmorPoints,
//             //                               position, rotation);
//             // Eigen::Vector3d position3d_camera(position.x, position.y, position.z);
//             // Eigen::Vector3d position3d_world = curr_pose_ * cam2imu_static_ * position3d_camera;

//             // // pitch弹道解算
//             // trajectory_transform_tool->solve(position3d_world, target_pitch, target_yaw);
//             // double target_distance = position3d_world.norm(); //水平距离
//             // target_yaw = rm_util::rad_to_deg(atan2(position3d_world(0, 0), position3d_world(1, 0)));

//             // rm_interfaces::msg::GimbalCmd gimbal_cmd;
//             // gimbal_cmd.id = gimbal_cmd_id++;
//             // gimbal_cmd.position.pitch = target_pitch + pitch_offset;
//             // gimbal_cmd.position.yaw = -(target_yaw + yaw_offset);
//             // gimbal_cmd_pub_->publish(gimbal_cmd);

//             // geometry_msgs::msg::PointStamped target_point;
//             // target_point.point.x = position3d_world(0) / 100;
//             // target_point.point.y = position3d_world(1) / 100;
//             // target_point.point.z = position3d_world(2) / 100;
//             // target_point.header.frame_id = "imu_link";
//             // point_pub_->publish(target_point);
//             cv::imshow("VidTest", temp);
//             cv::waitKey(1);

//         }

//         // else
//         // {
//         //     rm_interfaces::msg::GimbalCmd gimbal_cmd;
//         //     gimbal_cmd.id = gimbal_cmd_id++;
//         //     // gimbal_cmd.position.pitch = c_pitch;
//         //     // gimbal_cmd.position.yaw = c_yaw;
//         //     gimbal_cmd_pub_->publish(gimbal_cmd);
//         // }
// // #ifdef RM_DEBUG_MODE
// //         cv::imshow("target", temp);
// //         cv::waitKey(1);
// // #endif

//     }
// #endif

//     void NahsorNode::process_image(std_msgs::msg::Header header, cv::Mat &img, geometry_msgs::msg::Quaternion q)
//     {
//         int res = -1;
//         curr_pose_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
//         auto euler_angles = rm_util::CoordinateTranslation::quat2euler(curr_pose_);
//         float c_pitch, c_yaw, c_roll;
//         c_pitch = rm_util::rad_to_deg(euler_angles(0));
//         c_yaw = rm_util::rad_to_deg(euler_angles(2));
//         c_roll = rm_util::rad_to_deg(euler_angles(1));
//         (void)c_roll;

//         if (current_mode_ == TaskMode::small)
//         {
//             res = nahsor_algo->smallPred(img);
//         }
//         if (current_mode_ == TaskMode::large)
//         {
//             double time_stamp_s = header.stamp.sec + header.stamp.nanosec * 1e-9;
//             res = nahsor_algo->bigPred(img, time_stamp_s);
//         }
//         if (current_mode_ == TaskMode::test)
//         {
//             res = nahsor_algo->noPred(img);
//         }

//         if (res == 0)
//         {
//             auto point = nahsor_algo->get_target();
//             auto R_point = nahsor_algo->get_R_center();
// #ifdef RM_DEBUG_MODE
//             cv::Mat temp = img.clone();
//             cv::circle(temp, point, 10, cv::Scalar(0, 255, 0));
//             cv::circle(temp, R_point, 10, cv::Scalar(255, 0, 0));
// #endif
//             float pitch, yaw;
            
//             mono_location_tool->calc_view_angle(point, pitch, yaw);
//             yaw = rm_util::rad_to_deg(yaw);

//             // PNP解算
//             double target_pitch, target_yaw;
//             cv::Point3f position;
//             cv::Mat rotation;
//             std::vector<cv::Point2f> points;
//             points.push_back(nahsor_algo->get_points()[0]);
//             points.push_back(nahsor_algo->get_points()[1]);
//             points.push_back(nahsor_algo->get_points()[2]);
//             points.push_back(nahsor_algo->get_points()[3]);
//             mono_location_tool->solve_pnp(points, mBigArmorPoints,
//                                           position, rotation);
//             Eigen::Vector3d position3d_camera(position.x, position.y, position.z);
//             Eigen::Vector3d position3d_world = curr_pose_ * cam2imu_static_ * position3d_camera;

//             //tf变换，发布相机到世界的目标点tf信息
//             static tf2_ros::TransformBroadcaster pose_broadcaster_(node_);
//             //广播器消息类型实例化
//             geometry_msgs::msg::TransformStamped pose_tf;
//             //根据目标点位姿，发布对世界坐标系的变换
//             pose_tf.header.stamp = node_->now();
//             pose_tf.header.frame_id = "world";
//             pose_tf.child_frame_id = "target_point";
//             pose_tf.transform.translation.x = position.x;
//             pose_tf.transform.translation.y = position.y;
//             pose_tf.transform.translation.z = position.z;
//             //发布坐标变换
//             pose_broadcaster_.sendTransform(pose_tf);

//             // pitch弹道解算
//             trajectory_transform_tool->solve(position3d_world, target_pitch, target_yaw);
//             double target_distance = position3d_world.norm(); //水平距离
//             target_yaw = rm_util::rad_to_deg(atan2(position3d_world(0, 0), position3d_world(1, 0)));

//             rm_interfaces::msg::GimbalCmd gimbal_cmd;
//             gimbal_cmd.id = gimbal_cmd_id++;
//             gimbal_cmd.position.pitch = target_pitch + pitch_offset;
//             gimbal_cmd.position.yaw = -(target_yaw + yaw_offset);
//             gimbal_cmd_pub_->publish(gimbal_cmd);

//             geometry_msgs::msg::PointStamped target_point;
//             target_point.point.x = position3d_world(0) / 100;
//             target_point.point.y = position3d_world(1) / 100;
//             target_point.point.z = position3d_world(2) / 100;
//             target_point.header.frame_id = "imu_link";
//             point_pub_->publish(target_point);


// #ifdef RM_DEBUG_MODE
//             cv::imshow("VidTest", temp);
//             cv::waitKey(5);
// #endif
//         }

//         else
//         {
//             rm_interfaces::msg::GimbalCmd gimbal_cmd;
//             gimbal_cmd.id = gimbal_cmd_id++;
//             gimbal_cmd.position.pitch = c_pitch;
//             gimbal_cmd.position.yaw = c_yaw;
//             gimbal_cmd_pub_->publish(gimbal_cmd);
//         }
// // #ifdef RM_DEBUG_MODE
// //         cv::imshow("target", temp);
// //         cv::waitKey(1);
// // #endif
//     }
    bool NahsorNode::setModeCallBack(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
    {
        response->success = true;
        //小符
        if (request->mode == 0xbb)
        {
            current_mode_ = TaskMode::small;
            wrapper_client_->start();
            RCLCPP_INFO(node_->get_logger(), "【smell】!");
        }
        //大符
        else if (request->mode == 0xcc)
        {
            current_mode_ = TaskMode::large;
            wrapper_client_->start();
            RCLCPP_INFO(node_->get_logger(), "【big】!");
        }
        //测试用
        else if (request->mode == 0xdd)
        {
            current_mode_ = TaskMode::large;
            wrapper_client_->start();
        }
        //自瞄则关闭符代码
        else if (request->mode == 0x01 || request->mode == 0xaa)
        {
            wrapper_client_->stop();
        }
        return true;
    }
    bool NahsorNode::set_color_cb(const std::shared_ptr<rm_interfaces::srv::SetColor::Request> request,
                                  std::shared_ptr<rm_interfaces::srv::SetColor::Response> response)
    {
        response->success = true;
        int color = request->color;
        // wrapper_client_->stop();
        if (color == 0xbb)
        {
            nahsor_algo->set_target_color(false);
            RCLCPP_INFO(node_->get_logger(), "set target Color【BLUE】!");
        }
        else
        {
            nahsor_algo->set_target_color(true);
            RCLCPP_INFO(node_->get_logger(), "set target Color【RED】!");
        }
        // wrapper_client_->start();
        return true;
    }

}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nahsor::NahsorNode)