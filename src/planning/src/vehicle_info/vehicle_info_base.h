#ifndef PNC_MAP_BASE_H_
#define PNC_MAP_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "config_reader.h"
#include "curve.h"

namespace Planning
{
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using geometry_msgs::msg::TransformStamped;
    using nav_msgs::msg::Path;

    class VehicleBase // 车辆基类
    {
    public:
        // 更新参数
        inline void update_location(const PoseStamped &loc) { loc_point_ = loc; }

        // 定位转frenet
        virtual void vechicle_cartesin_to_frent(const Referline &refer_line) = 0 ; // 定位点在参考线上的投影点参数

        inline std::string child_frame() const { return child_frame_; }
        inline double length() const { return length_; }
        inline double width() const { return width_; }
        inline int id() const { return id_; }
        inline PoseStamped loc_point() const { return loc_point_; }
        inline double theta() const { return theta_; }
        inline double kappa() const { return kappa_; }
        inline double speed() const { return speed_; }
        inline double acceleration() const { return acceleration_; }
        inline double dacceleration() const { return dacceleration_; }

        // 向参考线投影的frenet参数
        inline ToFrenetInPutTP to_refline_frenet_params() const { return to_refline_frenet_params_; }
        inline ToFrenetOutTP to_path_frenet_params() const { return to_path_frenet_params_; }

        virtual ~VehicleBase() {}

    protected:
        // 基本属性
        std::unique_ptr<ConfigReader> vehicle_config_; // 配置
        std::string child_frame_;                      // 坐标名
        double length_ = 0.0;                          // 长
        double width_ = 0.0;                           // 宽
        int id_ = 0;                                   // 序号
        // 笛卡尔参数
        PoseStamped loc_point_;      // 车辆位姿
        double theta_ = 0.0;         // 航向角
        double kappa_ = 0.0;         // 曲率
        double speed_ = 0.0;         // 速度
        double acceleration_ = 0.0;  // 加速度
        double dacceleration_ = 0.0; // 纵向加加速度
        
        // 向参考线投影的frenet参数
        ToFrenetInPutTP to_refline_frenet_params_;
        // 向路径投影的frenet参数
        ToFrenetOutTP to_path_frenet_params_;
        // 时间参数
    };
} // namespace Planning
#endif // PNC_MAP_BASE_H_
