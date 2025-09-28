#include"obs_car_info.h"

namespace Planning{
    ObsCar::ObsCar()  // 障碍物车辆
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car created");
    }

}   //namespace Planning