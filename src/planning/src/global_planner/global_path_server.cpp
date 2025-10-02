#include "global_path_server.h"

namespace Planning
{
    GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "global_path_server_node created");

        // 创建全局路径规划器
        global_path_pub_ = this->create_publisher<Path>("global_path", 10);
        global_path_rviz_pub_ = this->create_publisher<Marker>("global_path_rviz", 10);
        // 创建全局路径服务
        global_path_server_ = this->create_service<GlobalPathService>(
            "global_path_service",
            std::bind(&GlobalPathServer::reponse_global_path_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    void GlobalPathServer::reponse_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                                        const std::shared_ptr<GlobalPathService::Response> response)
    {
        // 接受请求，多态
        switch (request->global_planner_type)
        {
        case static_cast<int>(GlobalPlannerType::NORMAL):
            global_planner_base_ = std::make_shared<GlobalPathNormal>();
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Invalid global_planner type");
            return;
        }
        // 判断请求是否有效
        if (request->pnc_map.midline.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "pnc_map empty, global_path connot be created");
            return;
        }
        // 搜索并相应全局路径
        const auto global_path = global_planner_base_->serch_global_path(request->pnc_map);
        response->global_path = global_path;

        // 发布全局路径，给局部规划用

        global_path_pub_->publish(global_path);
        RCLCPP_INFO(this->get_logger(), "global_path published");
        // 发布用于rviz显示的全局路径
        // bug修复：global_path 只发布一次 path没有frame_locked 无法固定在rviz中
        const auto global_path_rviz = path2marker(global_path);
        global_path_rviz_pub_->publish(global_path_rviz);
        RCLCPP_INFO(this->get_logger(), "global_path for rviz published");
    }

    Marker GlobalPathServer::path2marker(const Path &path)
    {
        Marker path_rviz_;
        return path_rviz_;
    }

} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}