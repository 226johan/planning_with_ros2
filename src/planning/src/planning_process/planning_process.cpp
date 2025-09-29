#include"planning_process.h"

namespace Planning{

    PlanningProcess::PlanningProcess() : Node("planning_node")  // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(),"planning_node created");

        // read config
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        auto obs_dis = process_config_->process().obs_dis_;
        // RCLCPP_INFO(this->get_logger(),"obs_dis: %.2f",obs_dis);

    }

    bool PlanningProcess::process(){    // 总流程
        return true;
    }
}
