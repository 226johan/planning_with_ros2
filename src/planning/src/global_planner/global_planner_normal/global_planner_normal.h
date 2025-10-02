#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include"global_planner_base.h"

namespace Planning
{
class GlobalPathNormal : public GlobalPlannerBase  // 普通全局路径规划器
{
public:
    GlobalPathNormal();
    Path serch_global_path(const PNCMap &pnc_map) override;

private:

};
}  // namespace Planning
#endif  // GLOBAL_PLANNER_NORMAL_H_
