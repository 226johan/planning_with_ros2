#include"pnc_map_server.h"

namespace Planning{
    PNCMapServer::PNCMapServer() : Node("pnc_map_server_node") //全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "pnc_map_server_node created");

        // 地图发布器
        map_pub_ = this->create_publisher<PNCMap>("pnc_map",10);
        map_rviz_pub_ = this->create_publisher<MarkerArray>("pnc_map_markerarray",10);
        // 地图服务器
        map_service_ = this->create_service<PNCMapService>(
            "pnc_map_server",
            std::bind(&PNCMapServer::response_pnc_map_callback,this,_1,_2)
        );
    }

    // 响应并发布地图
    void PNCMapServer::response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request, 
                                                const std::shared_ptr<PNCMapService::Response> response)
    {

    }

}   //namespace Planning

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}