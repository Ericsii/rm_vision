#ifndef NAHSOR_HPP
#define NAHSOR_HPP
#include <opencv2/opencv.hpp>
// #include "rm_nahsor/nahsor_detector.hpp"
#include "rm_nahsor/nahsor_prediction.hpp"
#include "rm_util/rm_util.hpp"
#include "rm_nahsor/rune.hpp"
#include "rm_auto_aim/armor_detector_interface.hpp"
#include "rm_util/debug_image.hpp"
#include "rune_detector/detector.hpp"
namespace nahsor
{
    class NahsorAlgo
    {
    public:
        NahsorAlgo(rclcpp::Node::SharedPtr node,
                   std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool);
        int smallPred(cv::Mat &img);


        int bigPred_new(cv::Mat &img, double t);

        int noPred(cv::Mat &img);
        /**
         * @brief 建立图模型
         * 
         */
        void graph();
        /**
         * @brief 给图添加边
         * 
         * @param angle 角度
         * @param time 时间戳
         */
        void addEdge(double angle, double time);
        /**
         * @brief 进行优化
         * 
         */
        void optimize();
        
        cv::Point2f get_target();

        cv::Point2f get_R_center();
        double get_angle();
        cv::Point2f* get_points();
        double target_pitch,target_yaw;
        void set_delay(double delta);
        void set_target_color(bool is_red);
        cv::Point3f getEular(cv::Point2f sorted_points[4], rm_power_rune::Circle circle);
        cv::Point3f rotationMatrixToEulerAngles(cv::Mat &R);
        bool isRotationMatrix(cv::Mat &R);
    private:
        rclcpp::Node::SharedPtr node_;

        rm_power_rune::Detector DetectorTest;
        
        std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool_;
        g2o::SparseOptimizer optimizer;
        CurveFittingVertex *v;
        double t0,w,a,b;//v = a * sin(w * t) + b
        int count{0};
        bool build_graph{false};
        float target_x,target_y;
        double delta_t = 0.55;
        double target_angle;
        //上次的时间和角度，用以计算速度
        double last_angle{-1};
        double last_angle_pnp{-1};
        double last_time{-1};
        double last_2_angle{-1};
        double last_2_time{-1};
        double last_v;
        //是否顺时针旋转
        bool is_clockwise;
        double temp_t{0};
        bool optimized{false};
        Rune armor;
        Rune pre_armor;
        std::vector<cv::Point3f> mBigArmorPoints; 
        Eigen::Quaterniond cam2imu_static_;
        Eigen::Quaterniond imu2cam_static_;
        std::vector<float> debug_x;
        std::vector<float> debug_y;
        std::vector<float> debug_x1;
        std::vector<float> debug_y1;
        int cnt;
        float alpha = 0.1;
        std::queue<float> q_time;
        std::queue<float> q_angle;
        float small_v;
    };
}
#endif