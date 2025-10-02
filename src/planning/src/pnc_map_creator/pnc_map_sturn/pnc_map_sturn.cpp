#include "pnc_map_sturn.h"

namespace Planning
{
    PNCMapCreatorSTurn::PNCMapCreatorSTurn() // sturn地图
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_uturn_create created");
    }
    PNCMap PNCMapCreatorSTurn::creat_pnc_map() // 生成地图
    {
        return pnc_map_;
    }

} // namespace Planning
