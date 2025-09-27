#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
  class PlanningProcess : public rclcpp::Node
  {
  public:
    PlanningProcess() : Node("planning_node")
    {

    }
    bool process(){return true;}; // 总流程

  private:

  };
}  // namespace Planning
#endif  // PLANNING_PROCESS_H_


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("planning_node"),"planning start");
  auto node = std::make_shared<Planning::PlanningProcess>();
  if(!node->process()){
    RCLCPP_ERROR(rclcpp::get_logger("planning_node"),"planning failed");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
