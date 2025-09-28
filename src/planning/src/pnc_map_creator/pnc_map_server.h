#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class PNCMapServer : public rclcpp::Node  // 全局路径规划器基类
    {
        public:
        PNCMapServer();

    };
}  // namespace Planning
#endif  // PNC_MAP_SERVER_H_