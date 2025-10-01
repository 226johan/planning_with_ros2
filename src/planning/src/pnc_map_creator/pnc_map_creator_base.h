#ifndef PNC_MAP_BASE_H_
#define PNC_MAP_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include"base_msgs/msg/pnc_map.hpp"
#include"visualization_msgs/msg/marker.hpp"
#include"visualization_msgs/msg/marker_array.hpp"
#include"config_reader.h"
namespace Planning
{
    using base_msgs::msg::PNCMap;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;
    class PNCMapCreatorBase  // pnc_map创建器基类
    {
        public:

    };
}  // namespace Planning
#endif  // PNC_MAP_BASE_H_
