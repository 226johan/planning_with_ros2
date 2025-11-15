#ifndef MAIN_CAR_INFO_H_
#define MAIN_CAR_INFO_H_

#include "vehicle_info_base.h"
#include "curve.h"
namespace Planning
{
    class MainCar : public VehicleBase // 主车
    {
    public:
        MainCar();

        void vechicle_cartesin_to_frent(const Referline &refer_line) override;
    };
} // namespace Planning
#endif // MAIN_CAR_INFO_H_