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
        // 接受请求 多态
        switch (request->map_type)
        {
        case static_cast<int>(PNCMapType::STRAIGHT):
            map_creater_ = std::make_shared<PNCMapCreatorStraight>();
            break;
        case static_cast<int>(PNCMapType::STURN):
            map_creater_ = std::make_shared<PNCMapCreatorSTurn>();
            break;
        default:
            break;
        }
        // 创建并相应地图
        const auto pnc_map = map_creater_->creat_pnc_map();
        response->pnc_map = pnc_map;
        // 发布地图 planning node
        map_pub_->publish(pnc_map);  // 发布地图
        RCLCPP_INFO(this->get_logger(), "pnc_map published");
        // 发布rviz显示的地图
        const auto pnc_map_makerarray = map_creater_->pnc_map_markerarray();
        map_rviz_pub_->publish(pnc_map_makerarray);

    }

}   //namespace Planning

int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}