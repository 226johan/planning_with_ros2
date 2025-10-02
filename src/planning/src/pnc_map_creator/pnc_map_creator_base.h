#ifndef PNC_MAP_BASE_H_
#define PNC_MAP_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "config_reader.h"
namespace Planning
{
    using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::Marker;
    using visualization_msgs::msg::MarkerArray;

    enum class PNCMapType // 地图类型
    {
        STRAIGHT,
        STURN,
    };

    class PNCMapCreatorBase // pnc_map创建器基类
    {
    public:
        virtual PNCMap creat_pnc_map() = 0;                                          // 生成地图
        inline PNCMap pnc_map() const { return pnc_map_; }                            // 获取地图
        inline MarkerArray pnc_map_markerarray() const { return pnc_map_makerarry_; } // 获取rviz显示的地图
        virtual ~PNCMapCreatorBase() {}

    protected:
        std::shared_ptr<ConfigReader> pnc_map_config_; // 配置
        int map_type_ = 0;                             // 类型
        PNCMap pnc_map_;                               // 地图
        MarkerArray pnc_map_makerarry_;                // rviz显示的地图

        Point p_mid_, pl_, pr_;                        // 中线，左线，右线
        double theta_current_ = 0.0;                   // 当前航向角
        double len_step_ = 0.0;                        // 步长
        double theta_step_ = 0.0;                      // 航向角步长
    };
} // namespace Planning
#endif // PNC_MAP_BASE_H_
