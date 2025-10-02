#ifndef PNC_MAP_STRAIGHT_H_
#define PNC_MAP_STRAIGHT_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorStraight : public PNCMapCreatorBase // 直道地图
    {
    public:
        PNCMapCreatorStraight();
        PNCMap creat_pnc_map() override; // 生成地图
    };
} // namespace Planning
#endif // PNC_MAP_STRAIGHT_H_