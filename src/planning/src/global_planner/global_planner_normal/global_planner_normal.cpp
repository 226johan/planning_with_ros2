#include"global_planner_normal.h"

namespace Planning{
    GlobalPathNormal::GlobalPathNormal(){   // 普通全局路径规划器
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_path_normal created");
    }
    Path GlobalPathNormal::serch_global_path(const PNCMap &pnc_map)
    {
        return global_path_;
    }

}  // namepace PlanninGlobalPathNormalobalPathNormal