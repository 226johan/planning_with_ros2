#include"obj_move_cmd.h"

namespace Planning
{
    ObjMoveCmd::ObjMoveCmd() : Node("obj_move_cmd_node"){   // 障碍物运动指令
        RCLCPP_INFO(this->get_logger(),"obj_move_cmd_node created");
    }

};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Planning::ObjMoveCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}