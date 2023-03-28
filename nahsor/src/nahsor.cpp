#include "rm_nahsor/nahsor.hpp"
#include "rm_auto_aim/armor_detector_interface.hpp"

#include <Eigen/Core>

double calc_dis(cv::Point2f &point1, cv::Point2f &point2)
{
    auto dis = (point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y);
    return sqrt(dis);
}
namespace nahsor
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    using rm_auto_aim::ArmorTarget;
    NahsorAlgo::NahsorAlgo(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<rm_util::MonoMeasureTool> mono_location_tool)
        : node_(node), mono_location_tool_(mono_location_tool), DetectorTest(node)
    {
        float realWidth, realHeight, half_x, half_y;
        realHeight = 6.3;
        realWidth = 22.2;
        half_x = realWidth / 2;
        half_y = realHeight / 2;
        mBigArmorPoints.emplace_back(-half_x, half_y, 0.);
        mBigArmorPoints.emplace_back(-half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, -half_y, 0);
        mBigArmorPoints.emplace_back(half_x, half_y, 0);
        node_->declare_parameter("small_v", 2.5);
        node_->get_parameter("small_v", small_v);
    }
    int NahsorAlgo::smallPred(cv::Mat &img)
    {

        bool r = DetectorTest.New_detect(img);
        auto armors = DetectorTest.get_armors();
        auto circle = DetectorTest.get_circle();
        // std::cout <<"sz: "<< armors.size() <<"r: "<< r << std::endl;
        if (r)
        {
            auto armor = armors;
            //暂时假设顺时针转动
            target_angle = armor.angle - small_v * delta_t;
            target_x = circle.R_center.x + circle.radius * cos(target_angle);
            target_y = circle.R_center.y + circle.radius * sin(target_angle);
            for (int i = 0; i < 4; i++)
            {
                auto angle = rm_util::deg_to_rad(rm_util::calc_inclination_angle(circle.R_center, armor.points[i]));
                auto dis = calc_dis(circle.R_center, armor.points[i]);
                auto pre_angle = angle - small_v * delta_t;
                pre_armor.points[i].x = circle.R_center.x + dis * cos(pre_angle);
                pre_armor.points[i].y = circle.R_center.y + dis * sin(pre_angle);
            }
            pre_armor.angle = target_angle;
            return 0;
        }
        return -1;
    }

    int NahsorAlgo::bigPred_new(cv::Mat &img, double t)
    {
        bool r = DetectorTest.New_detect(img);
        auto armors = DetectorTest.get_armors();
        // RCLCPP_ERROR(node_->get_logger(),"armors.size:%d",armors.size());
        auto circle = DetectorTest.get_circle();
        if (r==true)
        {
            t -= temp_t;
            auto armor = armors;
            if (true)
            {
                // rclcpp::sleep_for(std::chrono::seconds(20));
                if (!build_graph)
                {
                    this->graph();
                    last_angle = armor.angle;
                    temp_t = t;
                    last_time = 0;
                    build_graph = true;
                }
                else
                {
                    if (last_2_angle == -1)
                    {
                        last_2_angle = last_angle;
                        last_2_time = last_time;
                        last_angle = armor.angle;
                        last_time = t;
                        return 0;
                    }
                    auto diff_angle = armor.angle - last_angle;
                    last_2_angle = last_angle;
                    last_angle = armor.angle;

                    double temp = last_time;
                    last_2_time = last_time;
                    last_time = t;

                    //角度变化过大，说明出现跳变
                    //舍弃一些不合理的值
                    if (abs(diff_angle) > 0.10)
                    {
                        return 0;
                    }
                    if (count < 10)
                        is_clockwise = diff_angle < 0;
                    auto velocity = abs(diff_angle) / (t - temp);
                    if (velocity > 2)
                        return 0;

                    if (last_v != 0)
                        velocity = alpha * velocity + (1 - alpha) * last_v;
                    last_v = velocity;
                    if (count < 300)
                        this->addEdge(velocity, (t + temp) / 2);
                    count++;
                    target_x = circle.R_center.x;
                    target_y = circle.R_center.y;
                    if (count < 1000)
                    {
                        debug_x.push_back((t + temp) / 2);
                        debug_y.push_back(velocity);
                    }
                    else
                    {
                        cnt = count % 1000;
                        debug_x[cnt] = (t + temp) / 2;
                        debug_y[cnt] = velocity;
                    }
                    auto debug_img = rm_util::debug_image::draw_scatter(debug_x, debug_y, cv::Scalar(255, 0, 0), 1);
                    // cv::imshow("wave_img", debug_img);
                    // cv::waitKey(5);
                    // rclcpp::sleep_for(std::chrono::seconds(20));
                }
            }
            if (count == 300)
            {
                RCLCPP_INFO(node_->get_logger(), "开始优化");
                this->optimize();
                RCLCPP_INFO(node_->get_logger(), "a: %f, w: %f, t0: %f, b: %f", a, w, t0, b);
                optimized = true;
                count++;
            }
            if (optimized)
            {
                float dy = a * sin(w * (t + t0)) + b;
                float dx = t;
                int sz = debug_x1.size();
                if (sz < 1000)
                {
                    debug_x1.push_back(dx);
                    debug_y1.push_back(dy);
                }
                else
                {
                    int cnt1 = (count - 300) % 1000;
                    debug_x1[cnt1] = dx;
                    debug_y1[cnt1] = dy;
                }
                // auto debug_img2 = rm_util::debug_image::draw_scatter(debug_x1, debug_y1, cv::Scalar(255, 0, 0), 1);
                // cv::imshow("wave", debug_img2);
                // cv::waitKey(5);
                //根据转动方向
                auto change_angle = -(a / w) * (cos(w * (t + t0 + delta_t))) +
                                    (a / w) * (cos(w * (t + t0))) + b * delta_t;

                if (is_clockwise)
                    change_angle = -change_angle;

                target_angle = armor.angle + change_angle;

                //角度为0~2pi
                if (target_angle < 0)
                    target_angle += 2 * CV_PI;
                if (target_angle > 2 * CV_PI)
                    target_angle -= 2 * CV_PI;
                target_x = circle.R_center.x + circle.radius * cos(target_angle);
                target_y = circle.R_center.y + circle.radius * sin(target_angle);
                for (int i = 0; i < 4; i++)
                {
                    auto angle = rm_util::deg_to_rad(rm_util::calc_inclination_angle(circle.R_center, armor.points[i]));
                    auto dis = calc_dis(circle.R_center, armor.points[i]);
                    auto pre_angle = angle + change_angle;
                    pre_armor.points[i].x = circle.R_center.x + dis * cos(pre_angle);
                    pre_armor.points[i].y = circle.R_center.y + dis * sin(pre_angle);
                }
                pre_armor.angle = target_angle;
            }
            return 0;
        }
        return -1;
        
    }
//     int NahsorAlgo::bigPred_test(cv::Mat &img, double t)
//     {
//         // int r = nahsor_detector.detect(img);
//         //Test New Detector
//         // RCLCPP_ERROR(node_->get_logger(), "520");
//         // int r = DetectorTest.detect(img);
//         int r = DetectorTest.New_detect(img);
//         auto the_armor = DetectorTest.get_armors();
//         // RCLCPP_ERROR(node_->get_logger(),"armors.size:%d",armors.size());
//         auto circle = DetectorTest.get_circle();
//         if(std::abs(the_armor.armor_area-0)>0.5)
//         {
//             t -= temp_t;
//             auto armor = the_armor;
//             if (true)
//             {
//                 // rclcpp::sleep_for(std::chrono::seconds(20));
//                 if (!build_graph)
//                 {
//                     this->graph();
//                     // last_angle = armor.angle;

// //
//                     cv::Point3f pnpres = NahsorAlgo::getEular(armor.sorted_points, circle);
//                     last_angle_pnp = pnpres.z;

//                     last_angle = last_angle_pnp;

// //
                    
//                     // RCLCPP_ERROR(node_->get_logger(),"Armors.angle:%d",armor.angle);
//                     temp_t = t;
//                     last_time = 0;
//                     build_graph = true;
//                 }
//                 else
//                 {
//                     if (last_2_angle == -1)
//                     {
//                         last_2_angle = last_angle;
//                         cv::Point3f pnpres_2 = NahsorAlgo::getEular(armor.sorted_points, circle);
//                         last_angle_pnp = pnpres_2.z;
//                         last_angle = last_angle_pnp;
//                         last_2_time = last_time;
//                         // last_angle = armor.angle;
//                         last_time = t;
//                         return 0;
//                     }

//                     // auto diff_angle = armor.angle - last_angle;
// //
//                     cv::Point3f pnpres = NahsorAlgo::getEular(armor.sorted_points, circle);
//                     double angle_pnp = pnpres.z;

//                     auto diff_angle = angle_pnp - last_angle;

// //
//                     last_2_angle = last_angle;
//                     // last_angle = armor.angle;

// //
//                     last_angle = angle_pnp;
//                     // RCLCPP_ERROR(node_->get_logger(),"diff_angle:%f",diff_angle);

// //
//                     double temp = last_time;
//                     last_2_time = last_time;
//                     last_time = t;

//                     //角度变化过大，说明出现跳变
//                     //舍弃一些不合理的值
//                     if (abs(diff_angle) > 0.10)
//                     {
//                         return 0;
//                     }
//                     if (count < 10)
//                         is_clockwise = diff_angle < 0;
                    
//                     armor.D3_points[0] = {-0.13,0.05,0.0};
//                     armor.D3_points[1] = {0.13,0.05,0.0};
//                     armor.D3_points[2] = {0.13,-0.05,0.0};
//                     armor.D3_points[3] = {-0.13,-0.05,0.0};

//                     // cv::Point3f armor_angle_pnp = NahsorAlgo::getEular(armor.sorted_points);
//                     // RCLCPP_INFO(node_->get_logger(), "yaw:%f, pitch:%f, roll:%f", armor_angle_pnp.x, armor_angle_pnp.y, armor_angle_pnp.z);

//                     // double diff_angle_pnp = armor_angle_pnp.z - last_angle_pnp;
//                     auto velocity = abs(diff_angle) / (t - temp);
//                     // RCLCPP_ERROR(node_->get_logger(),"velocity:%f",velocity);
//                     if (velocity > 2)
//                         return 0;

//                     if (last_v != 0)
//                         velocity = alpha * velocity + (1 - alpha) * last_v;
//                     last_v = velocity;
//                     if (count < 300)
//                         this->addEdge(velocity, (t + temp) / 2);
//                     count++;
//                     target_x = circle.R_center.x;
//                     target_y = circle.R_center.y;
//                     if (count < 1000)
//                     {
//                         debug_x.push_back((t + temp) / 2);
//                         debug_y.push_back(velocity);
//                     }
//                     else
//                     {
//                         cnt = count % 1000;
//                         debug_x[cnt] = (t + temp) / 2;
//                         debug_y[cnt] = velocity;
//                     }
//                     auto debug_img = rm_util::debug_image::draw_scatter(debug_x, debug_y, cv::Scalar(255, 0, 0), 1);
//                     // cv::imshow("wave_img", debug_img);
//                     // cv::waitKey(5);
//                     // rclcpp::sleep_for(std::chrono::seconds(20));
//                 }
//             }
//             if (count == 300)
//             {
//                 RCLCPP_INFO(node_->get_logger(), "开始优化");
//                 this->optimize();
//                 RCLCPP_INFO(node_->get_logger(), "a: %f, w: %f, t0: %f, b: %f", a, w, t0, b);
//                 optimized = true;
//                 count++;
//             }
//             if (optimized)
//             {
//                 float dy = a * sin(w * (t + t0)) + b;
//                 float dx = t;
//                 int sz = debug_x1.size();
//                 if (sz < 1000)
//                 {
//                     debug_x1.push_back(dx);
//                     debug_y1.push_back(dy);
//                 }
//                 else
//                 {
//                     int cnt1 = (count - 300) % 1000;
//                     debug_x1[cnt1] = dx;
//                     debug_y1[cnt1] = dy;
//                 }
//                 auto debug_img2 = rm_util::debug_image::draw_scatter(debug_x1, debug_y1, cv::Scalar(255, 0, 0), 1);
//                 // cv::imshow("wave", debug_img2);
//                 // cv::waitKey(5);
//                 //根据转动方向
//                 auto change_angle = -(a / w) * (cos(w * (t + t0 + delta_t))) +
//                                     (a / w) * (cos(w * (t + t0))) + b * delta_t;

//                 if (is_clockwise)
//                     change_angle = -change_angle;
                
//                 // RCLCPP_INFO(node_->get_logger(), "armor_angle:%f",armor.angle);
                
//                 target_angle = armor.angle + change_angle;

//                 //角度为0~2pi
//                 if (target_angle < 0)
//                     target_angle += 2 * CV_PI;
//                 if (target_angle > 2 * CV_PI)
//                     target_angle -= 2 * CV_PI;
//                 target_x = circle.R_center.x + circle.radius * cos(target_angle);
//                 target_y = circle.R_center.y + circle.radius * sin(target_angle);
//                 // RCLCPP_INFO(node_->get_logger(), "t_x:%f, t_y:%f",target_x,target_y);
//                 for (int i = 0; i < 4; i++)
//                 {
//                     auto angle = rm_util::deg_to_rad(rm_util::calc_inclination_angle(circle.R_center, armor.points[i]));
//                     auto dis = calc_dis(circle.R_center, armor.points[i]);
//                     auto pre_angle = angle + change_angle;
//                     pre_armor.points[i].x = circle.R_center.x + dis * cos(pre_angle);
//                     pre_armor.points[i].y = circle.R_center.y + dis * sin(pre_angle);
//                 }
//                 pre_armor.angle = target_angle;

//             }
//             return 0;
//         }
//         return -1;
//     }

    // int NahsorAlgo::bigPred(cv::Mat &img, double t)
    // {
    //     int r = nahsor_detector.detect(img);
    //     //Test New Detector
    //     // int r = DetectorTest.detect(img);
    //     // rclcpp::sleep_for(std::chrono::seconds(20));

    //     auto armors = nahsor_detector.get_armors();
    //     // RCLCPP_ERROR(node_->get_logger(),"armors.size:%d",armors.size());
    //     auto circle = nahsor_detector.get_circle();
    //     if (armors.size() == 1 && r == 0)
    //     {
    //         t -= temp_t;
    //         auto armor = armors[0];
    //         if (true)
    //         {
    //             // rclcpp::sleep_for(std::chrono::seconds(20));
    //             if (!build_graph)
    //             {
    //                 this->graph();
    //                 last_angle = armor.angle;
    //                 temp_t = t;
    //                 last_time = 0;
    //                 build_graph = true;
    //             }
    //             else
    //             {
    //                 if (last_2_angle == -1)
    //                 {
    //                     last_2_angle = last_angle;
    //                     last_2_time = last_time;
    //                     last_angle = armor.angle;
    //                     last_time = t;
    //                     return 0;
    //                 }
    //                 auto diff_angle = armor.angle - last_angle;
    //                 last_2_angle = last_angle;
    //                 last_angle = armor.angle;

    //                 double temp = last_time;
    //                 last_2_time = last_time;
    //                 last_time = t;

    //                 //角度变化过大，说明出现跳变
    //                 //舍弃一些不合理的值
    //                 if (abs(diff_angle) > 0.10)
    //                 {
    //                     return 0;
    //                 }
    //                 if (count < 10)
    //                     is_clockwise = diff_angle < 0;
    //                 auto velocity = abs(diff_angle) / (t - temp);
    //                 if (velocity > 2)
    //                     return 0;

    //                 if (last_v != 0)
    //                     velocity = alpha * velocity + (1 - alpha) * last_v;
    //                 last_v = velocity;
    //                 if (count < 300)
    //                     this->addEdge(velocity, (t + temp) / 2);
    //                 count++;
    //                 target_x = circle.R_center.x;
    //                 target_y = circle.R_center.y;
    //                 if (count < 1000)
    //                 {
    //                     debug_x.push_back((t + temp) / 2);
    //                     debug_y.push_back(velocity);
    //                 }
    //                 else
    //                 {
    //                     cnt = count % 1000;
    //                     debug_x[cnt] = (t + temp) / 2;
    //                     debug_y[cnt] = velocity;
    //                 }
    //                 auto debug_img = rm_util::debug_image::draw_scatter(debug_x, debug_y, cv::Scalar(255, 0, 0), 1);
    //                 cv::imshow("wave_img", debug_img);
    //                 cv::waitKey(5);
    //                 // rclcpp::sleep_for(std::chrono::seconds(20));
    //             }
    //         }
    //         if (count == 300)
    //         {
    //             RCLCPP_INFO(node_->get_logger(), "开始优化");
    //             this->optimize();
    //             RCLCPP_INFO(node_->get_logger(), "a: %f, w: %f, t0: %f, b: %f", a, w, t0, b);
    //             optimized = true;
    //             count++;
    //         }
    //         if (optimized)
    //         {
    //             float dy = a * sin(w * (t + t0)) + b;
    //             float dx = t;
    //             int sz = debug_x1.size();
    //             if (sz < 1000)
    //             {
    //                 debug_x1.push_back(dx);
    //                 debug_y1.push_back(dy);
    //             }
    //             else
    //             {
    //                 int cnt1 = (count - 300) % 1000;
    //                 debug_x1[cnt1] = dx;
    //                 debug_y1[cnt1] = dy;
    //             }
    //             auto debug_img2 = rm_util::debug_image::draw_scatter(debug_x1, debug_y1, cv::Scalar(255, 0, 0), 1);
    //             cv::imshow("wave", debug_img2);
    //             cv::waitKey(5);
    //             //根据转动方向
    //             auto change_angle = -(a / w) * (cos(w * (t + t0 + delta_t))) +
    //                                 (a / w) * (cos(w * (t + t0))) + b * delta_t;

    //             if (is_clockwise)
    //                 change_angle = -change_angle;

    //             target_angle = armor.angle + change_angle;

    //             //角度为0~2pi
    //             if (target_angle < 0)
    //                 target_angle += 2 * CV_PI;
    //             if (target_angle > 2 * CV_PI)
    //                 target_angle -= 2 * CV_PI;
    //             target_x = circle.R_center.x + circle.radius * cos(target_angle);
    //             target_y = circle.R_center.y + circle.radius * sin(target_angle);
    //             for (int i = 0; i < 4; i++)
    //             {
    //                 auto angle = rm_util::deg_to_rad(rm_util::calc_inclination_angle(circle.R_center, armor.points[i]));
    //                 auto dis = calc_dis(circle.R_center, armor.points[i]);
    //                 auto pre_angle = angle + change_angle;
    //                 pre_armor.points[i].x = circle.R_center.x + dis * cos(pre_angle);
    //                 pre_armor.points[i].y = circle.R_center.y + dis * sin(pre_angle);
    //             }
    //             pre_armor.angle = target_angle;
    //         }
    //         return 0;
    //     }
    //     return -1;
    // }
    int NahsorAlgo::noPred(cv::Mat &img)
    {
        bool r = DetectorTest.New_detect(img);
        auto armors = DetectorTest.get_armors();
        if (r)
        {
            armor = armors;
            std::vector<cv::Point2f> points;
            cv::Point3f position;
            cv::Mat rotation;
            target_x = armor.center.x;
            target_y = armor.center.y;
            target_angle = armor.angle;
            for (int i = 0; i < 4; i++)
            {
                points.push_back(armor.points[i]);
                pre_armor.points[i] = armor.points[i];
            }
            return 0;
        }
        return -1;
    }
    void NahsorAlgo::graph()
    {

        auto solver = new g2o::OptimizationAlgorithmGaussNewton(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);
        v = new CurveFittingVertex();
        v->setEstimate(Eigen::Vector3d(1.045, 1.884, 0));
        v->setId(0);
        optimizer.addVertex(v);
    }
    void NahsorAlgo::addEdge(double velocity, double time)
    {
        double w_sigma = 1.0;
        CurveFittingEdge *edge = new CurveFittingEdge(time);
        edge->setId(1);
        edge->setVertex(0, v);
        edge->setMeasurement(velocity);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        optimizer.addEdge(edge);
    }
    void NahsorAlgo::optimize()
    {
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        auto res = v->estimate();
        a = res[0];
        w = res[1];
        t0 = res[2];
        b = 2.0 - a;
    }

    cv::Point2f NahsorAlgo::get_target()
    {
        return {target_x, target_y}; // z轴为固定值
    }

    cv::Point2f NahsorAlgo::get_R_center()
    {
        return DetectorTest.get_circle().R_center;
    }
    double NahsorAlgo::get_angle()
    {
        return target_angle;
    }

    cv::Point2f *NahsorAlgo::get_points()
    {
        return pre_armor.points;
    }

    void NahsorAlgo::set_delay(double delta)
    {
        delta_t = delta;
    }

    void NahsorAlgo::set_target_color(bool is_red)
    {
        DetectorTest.set_color(is_red);
    }

    cv::Point3f NahsorAlgo::getEular(cv::Point2f sorted_points[4], rm_power_rune::Circle circle)
    {
            Rune Armor_pnp;
            
            Armor_pnp.D3_points[0] = {-0.13,0.75,0.0};
            Armor_pnp.D3_points[1] = {0.13,0.75,0.0};
            Armor_pnp.D3_points[2] = {0.13,-0.65,0.0};
            Armor_pnp.D3_points[3] = {-0.13,-0.65,0.0};
            std::vector<cv::Point3f> D3p;
            D3p.push_back(Armor_pnp.D3_points[0]);
            D3p.push_back(Armor_pnp.D3_points[1]);
            D3p.push_back(Armor_pnp.D3_points[2]);
            D3p.push_back(Armor_pnp.D3_points[3]);
            std::vector<cv::Point2f> D2p;
            D2p.push_back(sorted_points[0]);
            D2p.push_back(sorted_points[1]);
            D2p.push_back(sorted_points[2]);
            D2p.push_back(sorted_points[3]);

            double fx = 800; //focal length x
            double fy = 800;//focal le
            
            double cx = 400; //optical centre x
            double cy = 500; //optical centre y
            
            cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
            cameraMatrix.at<double>(0,0)=fx;
            cameraMatrix.at<double>(1,1)=fy;
            cameraMatrix.at<double>(2,2)=1;
            cameraMatrix.at<double>(0,2)=cx;
            cameraMatrix.at<double>(1,2)=cy;
            cameraMatrix.at<double>(0,1)=0;
            cameraMatrix.at<double>(1,0)=0;
            cameraMatrix.at<double>(2,0)=0;
            cameraMatrix.at<double>(2,1)=0;
            
            
            //std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;
            
            cv::Mat distCoeffs(5,1,cv::DataType<double>::type);
            distCoeffs.at<double>(0) = -0.09;
            distCoeffs.at<double>(1) = 0.33;
            distCoeffs.at<double>(2) = 0;
            distCoeffs.at<double>(3) = 0;
            distCoeffs.at<double>(5) = -0.22;
            cv::Mat rvec(3,1,cv::DataType<double>::type);
            cv::Mat tvec(3,1,cv::DataType<double>::type);

            cv::solvePnP(D3p, D2p, cameraMatrix, distCoeffs, rvec, tvec);
            cv::Point3f vel_eular = NahsorAlgo::rotationMatrixToEulerAngles(rvec);
            return vel_eular;
    }

    bool NahsorAlgo::isRotationMatrix(cv::Mat &R)
    {
        cv::Mat Rt;
        transpose(R, Rt);
        cv::Mat shouldBeIdentity = Rt * R;
        cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
        return  norm(I, shouldBeIdentity) < 1e-6;
        
    }
 
    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).

    cv::Point3f NahsorAlgo::rotationMatrixToEulerAngles(cv::Mat &R)
    {
    
        assert(isRotationMatrix(R));
        
        float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    
        bool singular = sy < 1e-6; // If
    
        float x, y, z;
        if (!singular)
        {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        }
        else
        {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }
        return cv::Point3f(x, y, z);
    }


}