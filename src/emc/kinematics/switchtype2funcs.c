/* switchtype2funcs.c: template file for switchkins_type==2 kinematics
** works with rtpreempt only (not rtai)
**
** Example Usage:
**        LDIR is LinuxCNC git root directory
**        UDIR is user directory (not in LinuxCNC git tree)
**  1) $ cp LDIR/src/emc/kinematics/switchtype2funcs.c  UDIR/mytype2.c
**  2) $ edit   UDIR/mytype2.c as required
**  3) $ source LDIR/scripts/rip-environment
**  4) For genser-switchkins module use make command line option:
**     $ cd LDIR/src
**     $ user_genser_type2=UDIR/mytype2.c make && sudo make setuid
*/

// typical includes:
#include "kinematics.h"
#include "rtapi_math.h"

// Add for kins based on genserkins:
// #include "genserkins.h" //includes gomath,hal

//**********************************************************************
// static local variables and functions go here

static int switchtype2_inited = 0;

//**********************************************************************
int switchtype2KinematicsSetup(char* coordinates,
                               int   max_joints)
{
    rtapi_print("\nswitchtype2KinematicsSetup:\n"
                  "   %s <%s> max_joints=%d\n\n",
                __FILE__,coordinates,max_joints);
    switchtype2_inited = 1;
    return 0; // 0 ==> OK
}

int switchtype2KinematicsForward(const double *joint,
                                 struct EmcPose * world,
                                 const KINEMATICS_FORWARD_FLAGS * fflags,
                                 KINEMATICS_INVERSE_FLAGS * iflags)
{
    if (!switchtype2_inited) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "switchtype2Kinematics: not initialized\n");
        return -1;
    }
    return identityKinematicsForward(joint,world,fflags,iflags);
}

int switchtype2KinematicsInverse(const EmcPose * pos,
                                 double *joint,
                                 const KINEMATICS_INVERSE_FLAGS * iflags,
                                 KINEMATICS_FORWARD_FLAGS * fflags)
{
    return identityKinematicsInverse(pos,joint,iflags,fflags);
}
