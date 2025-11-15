#ifndef OBS_CAR_INFO_H_
#define OBS_CAR_INFO_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class ObsCar : public VehicleBase  // 障碍物车辆
    {
        public:
        ObsCar(const int &id);

        void vechicle_cartesin_to_frent(const Referline &refer_line) override;

    };
}  // namespace Planning
#endif  // OBS_CAR_INFO_H_