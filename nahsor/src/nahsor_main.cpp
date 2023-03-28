/*
 * @Author: your name
 * @Date: 2021-11-30 20:00:14
 * @LastEditTime: 2021-11-30 21:29:15
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /scurm/nahsor_workspace/scu_rm_ros/contrib_modules/nahsor_modules/src/nahsor_main.cpp
 */
#include "rm_nahsor/nahsor_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node  = std::make_shared<nahsor::NahsorNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}