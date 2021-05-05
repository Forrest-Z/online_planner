#ifndef MP_AGNOSTIC_WRAPPER_H_
#define MP_AGNOSTIC_WRAPPER_H_

#include <local_planner/motion_primitives_planner.h>
#include <wrapper/base_wrapper.h>
#include <depth2pc/depth_to_pc.h>

namespace online_planner{
class MpAgnosticWrapper : public BaseWrapper{
public:
    MpAgnosticWrapper();
    ~MpAgnosticWrapper();
    int run_map_and_viz() override;
    int run_global_planner() override;
    int run_local_planner() override;
    int run_setpoint_publisher() override;





};
}

#endif