#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"

namespace Planning
{
    using base_msgs::msg::Referline;
    using base_msgs::msg::ReferlinePoint;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    class ReferenceLineCreator // 创建参考线
    {
    public:
        ReferenceLineCreator();

        Referline create_reference_line(const Path &global_path,
                                        const PoseStamped &target_point); // 生成参考线
        Path reference_to_rviz();

        void init_reference_line();

    public:
        inline Referline refer_line() const
        {
            return refer_line_;
        }
        inline Path refer_line_rviz() const { return refer_line_rviz_; }
        inline int match_point_index() const { return match_point_index_; }
        inline int front_index() const { return front_index_; }
        inline int back_index() const { return back_index_; }

    private:
        std::unique_ptr<ConfigReader> reference_line_config_;            // 配置
        Referline refer_line_;                                           // 参考线
        Path refer_line_rviz_;                                           // 参考线rviz
        std::shared_ptr<ReferenceLineSmoother> reference_line_smoother_; // 参考线平滑器
        int last_match_point_index_ = -1;                                // 上次匹配点在全局路径下标
        int match_point_index_ = -1;                                     // 当前匹配点在全局路径下标
        int front_index_ = -1;                                           // 最前点在全局路径下标
        int back_index_ = -1;                                            // 最后点在全局路径下标
    };
} // namespace Planning
#endif // REFERENCE_LINE_CREATOR_H_
