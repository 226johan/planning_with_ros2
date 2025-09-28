#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include"global_planner_normal.h"

namespace Planning
{
    class GlobalPathServer : public rclcpp::Node  // 全局路径规划器基类
    {
        public:
        GlobalPathServer();

    };
}  // namespace Planning
#endif  // GLOBAL_PATH_SERVER_H_