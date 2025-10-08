#include"pnc_map_topo.h"

namespace Planning
{
    PNCMapCreatorTopo::PNCMapCreatorTopo() // topo地图
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_topo_create created");
        // 读取配置文件，给参数赋值
        pnc_map_config_ = std::make_unique<ConfigReader>();
        pnc_map_config_->read_pnc_map_config();
        map_type_ = static_cast<int>(PNCMapType::TOPO);
    }

    PNCMap PNCMapCreatorTopo::creat_pnc_map()
    {
        len_step_ = 1.0;
        theta_step_ = 0.01;
        std::string topo_type = "grid";

        if(topo_type == "grid"){
            int rows = 5;
            int cols = 5;
            double cell_size = 10.0;
            create_grid_topo(rows,cols,cell_size);
        }
        else{
            create_grid_topo(5,5,10.0);
        }
        create_makers();

        return pnc_map_;
    }

    void PNCMapCreatorTopo::create_grid_topo(int rows, int cols, double cell_size)
    {
        for(int i=0;i<rows;++i){
            for(int j =0;j<cols;++j){
                Point pos;
                pos.x = j*cell_size;
                pos.y = i*cell_size;

                std::string node_type = "normal";
                if (i == 0 && j == 0) node_type = "start";
                if (i == rows-1 && j == cols-1) node_type = "goal";
                int node_id = add_node(pos,node_type);
            }
        }

        // 连接相邻节点
        for(int i=0;i<rows;++i){
            for(int j =0;j<cols;++j){
                int current_id = i*cols+j;
                
                // 连接右边节点
                if(j<cols-1){
                    int right_id = current_id+1;
                    connect_nodes(current_id,right_id,"horizontal");
                }

                // 连接下边节点
                if(i<rows-1){
                    int down_id = current_id+cols;
                    connect_nodes(current_id,down_id,"vertical");
                }
            }
        }

    }

    int PNCMapCreatorTopo::add_node(const Point& position, const std::string& type)
    {
        int node_id = nodes_.size();
        TopoNode node(node_id, position, type);
        nodes_.push_back(node);
        
        // 同时将节点位置添加到PNCMap中 
        pnc_map_.midline.points.emplace_back(position);
        
        return node_id;

    }
    
    void PNCMapCreatorTopo::add_edge(int from_node_id, int to_node_id, double cost, const std::string& type)
    {
        if (from_node_id >= 0 && from_node_id < static_cast<int>(nodes_.size()) &&
        to_node_id >= 0 && to_node_id < static_cast<int>(nodes_.size())) {
        
        TopoEdge edge(from_node_id, to_node_id, cost, type);
        edges_.push_back(edge);
        
        // 更新节点的邻居信息
        nodes_[from_node_id].neighbors.push_back(to_node_id);
        nodes_[from_node_id].edge_costs[to_node_id] = cost;
        
        // 如果是无向图，也添加反向边
        nodes_[to_node_id].neighbors.push_back(from_node_id);
        nodes_[to_node_id].edge_costs[from_node_id] = cost;
    }

    }
    
    void PNCMapCreatorTopo::connect_nodes(int node_id1,int node_id2,const std::string& type)
    {
        if (node_id1 >= 0 && node_id1 < static_cast<int>(nodes_.size()) &&
        node_id2 >= 0 && node_id2 < static_cast<int>(nodes_.size())) {
        
        double cost = calculate_distance(nodes_[node_id1].position, nodes_[node_id2].position);
        add_edge(node_id1, node_id2, cost, type);
    }

    }

    double PNCMapCreatorTopo::calculate_distance(const Point& p1,const Point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return sqrt(dx * dx + dy * dy);

    }

    void PNCMapCreatorTopo::create_makers()
    {
        // 创建节点标记
        for (const auto& node : nodes_) {
            Marker node_marker = node_maker(node);
            pnc_map_makerarry_.markers.emplace_back(node_marker);
        }
        
        // 创建边标记
        for (const auto& edge : edges_) {
            Marker edge_marker = edge_maker(edge);
            pnc_map_makerarry_.markers.emplace_back(edge_marker);
        }
    }
    Marker PNCMapCreatorTopo::node_maker(const TopoNode& node)
    {
        Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "topo_nodes";
        marker.id = node.id;
        marker.type = Marker::SPHERE;
        marker.action = Marker::ADD;
        
        marker.pose.position = node.position;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        
        // 根据节点类型设置颜色
        if (node.type == "start") {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        } 
        else if (node.type == "goal") {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else if (node.type == "center") {
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else {
            marker.color.r = 0.5f;
            marker.color.g = 0.5f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        
        return marker;

    }
    
    Marker PNCMapCreatorTopo::edge_maker(const TopoEdge& edge)
    {
        Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "topo_edges";
        marker.id = edge.from_node_id * 1000 + edge.to_node_id; // 确保ID唯一
        marker.type = Marker::LINE_LIST;
        marker.action = Marker::ADD;
        
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.2;
        
        // 根据边类型设置颜色
        if (edge.type == "horizontal") {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else if (edge.type == "vertical") {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else if (edge.type == "ring") {
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else {
            marker.color.r = 0.7f;
            marker.color.g = 0.7f;
            marker.color.b = 0.7f;
            marker.color.a = 1.0f;
        }
        
        // 设置线段端点
        marker.points.push_back(nodes_[edge.from_node_id].position);
        marker.points.push_back(nodes_[edge.to_node_id].position);
        
        return marker;

    }


}