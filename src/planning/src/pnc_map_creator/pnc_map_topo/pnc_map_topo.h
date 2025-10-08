#ifndef PNC_MAP_TOPO_H_
#define PNC_MAP_TOPO_H_

#include "pnc_map_creator_base.h"
#include<vector>
#include<map>
#include<memory>

namespace Planning
{

    struct TopoNode{
    int id;                              // 节点ID
    Point position;                      // 节点位置
    std::vector<int> neighbors;          // 邻居节点ID列表
    std::map<int, double> edge_costs;    // 到邻居节点的代价
    std::string type;                    // 节点类型（路口、路段等）

    TopoNode():id(-1){}
    TopoNode(int node_id,Point pos,const std::string& node_type = "normal")
        : id(node_id),position(pos),type(node_type){}

    };

    struct TopoEdge{
    int from_node_id;                    // 起始节点ID
    int to_node_id;                      // 终止节点ID
    double cost;                         // 边的代价
    std::string type;                    // 边类型（直行、转弯等）
    
    TopoEdge(int from, int to, double edge_cost, const std::string& edge_type = "normal")
        : from_node_id(from), to_node_id(to), cost(edge_cost), type(edge_type) {}

    };

    class PNCMapCreatorTopo : public PNCMapCreatorBase // topo地图
    {
    public:
        PNCMapCreatorTopo();
        PNCMap creat_pnc_map() override; // 生成地图

        inline std::vector<TopoNode> nodes() const { return nodes_; }
        inline std::vector<TopoEdge> edges() const { return edges_; }

    private:
        int add_node(const Point& position, const std::string& type = "normal"); // 添加节点
        void add_edge(int from_node_id, int to_node_id, double cost, const std::string& type = "normal"); // 添加边
        void connect_nodes(int node_id1,int node_id2,const std::string& type = "normal");
        double calculate_distance(const Point& p1,const Point& p2);

    void create_grid_topo(int rows, int cols, double cell_size);        // 创建网格状拓扑结构
        void create_makers();
        Marker node_maker(const TopoNode& node);
        Marker edge_maker(const TopoEdge& edge);

    private:
        void init_pnc_map();                                                                            // 初始化地图
        std::vector<TopoNode> nodes_;
        std::vector<TopoEdge> edges_;
    };
} // namespace Planning
#endif // PNC_MAP_TOPO_H_