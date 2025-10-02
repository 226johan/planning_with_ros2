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
    };
} // namespace Planning
#endif // PNC_MAP_UTURN_H_