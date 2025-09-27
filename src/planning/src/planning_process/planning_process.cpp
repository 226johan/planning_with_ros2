#include"planning_process.h"

namespace Planning{

    PlanningProcess::PlanningProcess() : Node("planning_node")  // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(),"planning_node created");
    }

    bool PlanningProcess::process(){    // 总流程
        return true;
    }
}
