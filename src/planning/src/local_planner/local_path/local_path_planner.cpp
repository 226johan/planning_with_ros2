#include "local_path_planner.h"

namespace Planning
{
    LocalPathPlanner::LocalPathPlanner()
    { // 局部路径规划器
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_planner created");

        local_path_config_ = std::make_unique<ConfigReader>();
        local_path_config_->read_local_path_config();

        local_path_smoother_ = std::shared_ptr<LocalPathSmoother>();
    }
    LocalPath LocalPathPlanner::creat_local_path(const Referline &reference_line,
                                                 const std::shared_ptr<VehicleBase> &car,
                                                 const std::shared_ptr<DecisionCenter> &decision)
    {
        init_local_path();

        // 计算sl
        double point_s = car->to_path_frenet_params().s;
        LocalPathPoint point_tmp;
        for (int i = 0; i < local_path_config_->local_path().path_size_; i++)
        {
            // 规划起点
            point_s += car->to_path_frenet_params().ds_dt; // 路径规划起点：下一帧参考线投影点的s m/帧
            if (point_s > reference_line.refer_line.back().rs) // 投影点超过参考线最前端，退出
            {
                break;
            }
            // 给point_tmp赋值，先给s ds_dt赋上，l和dl_ds初始化
            point_tmp.s = point_s;
            point_tmp.ds_dt = car->to_path_frenet_params().ds_dt;
            point_tmp.dds_dt = car->to_path_frenet_params().dds_dt;
            point_tmp.l = 0.0;
            point_tmp.dl_ds = 0.0;
            point_tmp.ddl_ds = 0.0;

            // 计算point_tmp的l和dl_ds
            const int sl_points_size = decision->sl_points().size();
            for (int j = 0; j < sl_points_size - 1; j++)
            {
                // 确定分段起始状态和末状态
                const double start_s = decision->sl_points()[j].s_;
                const double start_l = decision->sl_points()[j].l_;
                const double start_dl_ds = 0.0;
                const double start_ddl_ds = 0.0;

                const double end_s = decision->sl_points()[j + 1].s_;
                const double end_l = decision->sl_points()[j + 1].l_;
                const double end_dl_ds = 0.0;
                const double end_ddl_ds = 0.0;

                // 如果临时点在s的分段范围内
                if (point_s >= start_s && point_s < end_s)
                {
                    // 五次多项式
                    const double point_s_2 = point_s * point_s;
                    const double point_s_3 = point_s_2 * point_s;
                    const double point_s_4 = point_s_3 * point_s;
                    const double point_s_5 = point_s_4 * point_s;
                    const Eigen::Vector<double, 6> a = PolynomialCurve::quintic_polynomial(start_s, start_l, start_dl_ds, start_ddl_ds,
                                                                                           end_s, end_l, end_dl_ds, end_ddl_ds);
                    point_tmp.l = a(0) + a(1) * point_s + a(2) * point_s_2 + a(3) * point_s_3 + a(4) * point_s_4 + a(5) * point_s_5;
                    point_tmp.dl_ds = a(1) + 2.0 * a(2) * point_s + 3.0 * a(3) * point_s_2 + 4.0 * a(4) * point_s_3 + 5.0 * a(5) * point_s_4;
                    point_tmp.ddl_ds = a(2) + 6.0 * a(3) * point_s + 12.0 * a(4) * point_s_2 + 20.0 * a(5) * point_s_3;
                }
            }

            local_path_.local_path.emplace_back(point_tmp);
        }

        // 平滑 todo: johan
        local_path_smoother_->smooth_local_path(local_path_);

        // 转笛卡尔坐标系
        tf2::Quaternion qtn;
        for (auto &point : local_path_.local_path)
        {
            // 计算路径点在参考线上的投影
            const double rs = point.s;
            const int match_index = Curve::find_match_point(reference_line, rs);
            const double rx = reference_line.refer_line[match_index].pose.pose.position.x;
            const double ry = reference_line.refer_line[match_index].pose.pose.position.y;
            const double rtheta = reference_line.refer_line[match_index].rtheta;
            const double rkappa = reference_line.refer_line[match_index].rkappa;
            const double rdkappa = reference_line.refer_line[match_index].rdkappa;

            ToCartensianInPutTP ft_to_ct_input;
            ft_to_ct_input.s = point.s;
            ft_to_ct_input.ds_dt = point.ds_dt;
            ft_to_ct_input.dds_dt = point.dds_dt;
            ft_to_ct_input.l = point.l;
            ft_to_ct_input.dl_ds = point.dl_ds;
            ft_to_ct_input.ddl_ds = point.ddl_ds;

            ToCartensianOutTP ft_to_ct_output;
            ft_to_ct_output.rs = rs;
            ft_to_ct_output.rx = rx;
            ft_to_ct_output.ry = ry;
            ft_to_ct_output.rtheta = rtheta;
            ft_to_ct_output.rkappa = rkappa;
            ft_to_ct_output.rdkappa = rdkappa;
            // 计算路径点在笛卡尔下的参数
            Curve::frenet_to_cartensian(ft_to_ct_input, ft_to_ct_output);

            point.pose.header = local_path_.header;
            point.pose.pose.position.x = ft_to_ct_output.x;
            point.pose.pose.position.y = ft_to_ct_output.y;
            point.theta = ft_to_ct_output.theta;
            point.kappa = ft_to_ct_output.kappa;

            qtn.setRPY(0.0, 0.0, point.theta);
            point.pose.pose.orientation.x = qtn.getX();
            point.pose.pose.orientation.y = qtn.getY();
            point.pose.pose.orientation.z = qtn.getZ();
            point.pose.pose.orientation.w = qtn.getW();
        }

        // 计算投影点坐标
        Curve::cal_projection_param(local_path_);

        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local path created, size: %ld", local_path_.local_path.size());
        return local_path_;
    }
    Path LocalPathPlanner::path_to_rviz()
    {
        local_path_rviz_.header = local_path_.header;
        local_path_rviz_.poses.clear();
        PoseStamped point_tmp;
        point_tmp.header = local_path_rviz_.header;
        for (const auto &point : local_path_.local_path)
        {
            point_tmp.pose = point.pose.pose;
            local_path_rviz_.poses.push_back(point_tmp);
        }

        return local_path_rviz_;
    }
    void LocalPathPlanner::init_local_path()
    {
        local_path_.header.frame_id = local_path_config_->pnc_map().frame_;
        local_path_.header.stamp = rclcpp::Clock().now();
        local_path_.local_path.clear();
    }
} // namespace Planning
