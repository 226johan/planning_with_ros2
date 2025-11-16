#include "decision_center.h"

namespace Planning
{
    DecisionCenter::DecisionCenter()
    {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "decision_center created");

        // 读取配置文件
        decision_config_ = std::make_unique<ConfigReader>();
        decision_config_->read_decision_config();
    }

    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car, const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        if (obses.empty())
        {
            return;
        }
        // 初始化
        sl_points_.clear();
        const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;              // 道路左边界
        const double right_bound_l = -decision_config_->pnc_map().road_half_width_ * 0.5;            // 道路右边界
        const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ - 50); // 开始考虑障碍物的范围  提前size-50个点
        const double least_length = std::max(car->to_path_frenet_params().ds_dt * dis_time, 30.0);   // 最小变道距离 根据车速调整距离，增加最近约束
        const double referline_end_length = decision_config_->refer_line().front_size_ *
                                            decision_config_->pnc_map().segment_len_; // 参考线前端的长度最大值
        SLPoint p;
        // 对每个障碍物计算变道点位
        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->to_path_frenet_params().s - car->to_path_frenet_params().s; // 与障碍物距离
            if (obs_dis_s > referline_end_length ||                                                   // 障碍物在参考线末端的距离 （接近目标终点，参考线变短，也要通过最长距离判断）
                obs_dis_s < -least_length)                                                            // 后方比较远的障碍物
            {
                continue;
            }
            if (obs->to_path_frenet_params().l > right_bound_l && obs->to_path_frenet_params().l < left_bound_l &&                                     // 障碍物在车道横向中间
                fabs(obs->to_path_frenet_params().dl_dt) < min_speed && obs->to_path_frenet_params().ds_dt < car->to_path_frenet_params().ds_dt / 2.0) // 障碍物横向速度为0，纵向速度慢
            {
                p.s_ = obs->to_path_frenet_params().s + obs->to_path_frenet_params().ds_dt * obs_dis_s / (car->to_path_frenet_params().ds_dt - obs->to_path_frenet_params().ds_dt);
                const double obs_left_bound_l = obs->to_path_frenet_params().l + obs->width() / 2.0;  // 障碍物左边界
                const double obs_right_bound_l = obs->to_path_frenet_params().l - obs->width() / 2.0; // 障碍物右边界
                const double left_width = left_bound_l - obs_left_bound_l;                            // 左边宽度
                const double right_width = obs_right_bound_l - right_bound_l;                         // 右边宽度

                if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 左边可以通过
                {
                    p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
                    p.type_ = static_cast<int>(SLPointType::LEFT_PASS);
                    sl_points_.push_back(p);
                }
                else
                {

                    if (right_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 右边可以通过
                    {
                        p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
                        p.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
                        sl_points_.push_back(p);
                    }
                    else // 两边宽度都不够
                    {
                        p.l_ = 0.0;
                        p.s_ = obs->to_path_frenet_params().s - decision_config_->decision().safe_dis_s_;
                        p.type_ = static_cast<int>(SLPointType::STOP);
                        sl_points_.push_back(p);
                        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "stop obs p:(s = %.2f, l = %.2f)", p.s_, p.l_);
                        break; // 更前面的不考虑
                    }
                }
            }
        }
        if (sl_points_.empty())
        {
            return;
        }
        // 头尾的处理
        SLPoint p_start;
        p_start.s_ = sl_points_[0].s_ - least_length;
        p_start.l_ = 0.0;
        p_start.type_ = static_cast<int>(SLPointType::START);
        sl_points_.emplace(sl_points_.begin(), p_start); // 头插
        if (sl_points_.back().type_ != static_cast<int>(SLPointType::END))
        {
            SLPoint p_end;
            p_end.s_ = sl_points_.back().s_ + least_length;
            p_end.l_ = 0.0;
            p_end.type_ = static_cast<int>(SLPointType::END);
            sl_points_.emplace_back(p_end);
        }
    }

} // namespace Planning
