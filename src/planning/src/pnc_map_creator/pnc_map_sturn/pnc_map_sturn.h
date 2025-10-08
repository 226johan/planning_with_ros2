#ifndef PNC_MAP_UTURN_H_
#define PNC_MAP_UTURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorSTurn : public PNCMapCreatorBase // 直道地图
    {
    public:
        PNCMapCreatorSTurn();
        PNCMap creat_pnc_map() override; // 生成地图

    private:
        void init_pnc_map();                                                                            // 初始化地图
        void draw_straight_x(const double &lenght, const double &plus_flag, const double &ratio = 1.0); // 沿x轴画直线
        void draw_arc(const double &angle, const double &plus_flag, const double &ratio = 1.0);         // 画弧线
    };
} // namespace Planning
#endif // PNC_MAP_UTURN_H_