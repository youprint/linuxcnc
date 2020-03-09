#include "genhexkins.h"
#include "rtapi_app.h"

KINS_NOT_SWITCHABLE;
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

KINEMATICS_TYPE kinematicsType()
{
  return KINEMATICS_BOTH;
}

int kinematicsForward(const double *joint,
                      EmcPose * world,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
    int r;
    r = genhexKinematicsForward(joint, world, fflags, iflags);
    if (r) return r;

    r = genhex_gui_forward_kins(joint,world);

    return r;
}

int kinematicsInverse(const EmcPose * world,
                      double *joints,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    return genhexKinematicsInverse(world, joints, iflags, fflags);
}

static int comp_id;

int rtapi_app_main(void)
{
    comp_id = hal_init("genhexkins");
    if (comp_id < 0) return comp_id;

    if (genhex_hal_setup(comp_id)) {
        hal_exit(comp_id);
        return -1;
    }

    hal_ready(comp_id);
    return 0;
}

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
