#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning
{
  using namespace std::chrono_literals;
  using base_msgs::msg::PNCMap;
  using base_msgs::srv::GlobalPathService;
  using base_msgs::srv::PNCMapService;
  using geometry_msgs::msg::PoseStamped;
  using nav_msgs::msg::Path;
  using tf2_ros::Buffer;
  using tf2_ros::StaticTransformBroadcaster;
  using tf2_ros::TransformListener;

  class PlanningProcess : public rclcpp::Node
  {
  public:
    PlanningProcess();
    bool process(); // 总流程

  private:
    bool planning_init();                                            // 初始化
    void vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle); // 生成车辆
    void get_location(const std::shared_ptr<VehicleBase> &vehicle);  // 获取位置

    template <typename T>
    bool connect_server(const T &client); // 连接服务器
    bool map_request();                   // 地图请求
    bool global_path_request();           // 全局路径请求
    void planning_callback();             // 总流程回调

  public:
    inline PNCMap pnc_map() const { return pnc_map_; }
    inline Path global_path() const { return global_path_; }

  private:
    std::unique_ptr<ConfigReader> process_config_; // 配置
    std::shared_ptr<VehicleBase> car_;             // 主车
    double obs_dis_;                               // 考虑障碍物的距离

    std::shared_ptr<StaticTransformBroadcaster> tf_broadcaster_; // 坐标广播
    std::unique_ptr<Buffer> buffer_;                             // tf2 buffer
    std::unique_ptr<TransformListener> tf_listener_;             // 位置监听

    PNCMap pnc_map_;                                                  // 地图
    Path global_path_;                                                // 全局路径
    rclcpp::Client<PNCMapService>::SharedPtr map_client_;             // 地图请求客户端
    rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_; // 全局地图请求客户端

    std::shared_ptr<ReferenceLineCreator> refer_line_creator_; // 参考线生成器
    rclcpp::Publisher<Path>::SharedPtr refer_line_pub_;        // 参考线发布器

    rclcpp::TimerBase::SharedPtr timer_; // 定时器
  };
} // namespace Planning
#endif // PLANNING_PROCESS_H_
