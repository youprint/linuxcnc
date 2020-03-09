/********************************************************************
TEST: switchable kinematics: identity or genhexkins

1) genhex kinematics provided by genhexfuncs.c (shared with genhexkins.c
2) uses same pin names as genhexkins

*/

#include "rtapi.h"
#include "rtapi_app.h"
#include <rtapi_string.h>
#include "genhexkins.h"
#include "motion.h"     // EMCMOT_MAX_JOINTS

//*********************************************************************
struct switchdata {
    hal_bit_t *kinstype_is_0;
    hal_bit_t *kinstype_is_1;
    hal_bit_t *kinstype_is_2;
} *switchdata = 0;

static hal_u32_t  switchkins_type;
// number of supported switchkins_types:
#define GENHEX_KTYPES 3
// Note: genhexKinematicsForward() is iterative and requires
//       an initial EmcPose -- use the last pose for that 
//       switchkins_type
//       (ok for identity kinematics too, pose not reqd)
static EmcPose lastpose[GENHEX_KTYPES];
static int use_lastpose[GENHEX_KTYPES];

static void save_lastpose(int ktype, EmcPose* pos)
{
    lastpose[ktype].tran.x = pos->tran.x;
    lastpose[ktype].tran.y = pos->tran.y;
    lastpose[ktype].tran.z = pos->tran.z;
    lastpose[ktype].a      = pos->a;
    lastpose[ktype].b      = pos->b;
    lastpose[ktype].c      = pos->c;
    lastpose[ktype].u      = pos->u;
    lastpose[ktype].v      = pos->v;
    lastpose[ktype].w      = pos->w;
} // save_lastpose()

static void get_lastpose(int ktype, EmcPose* pos)
{
    pos->tran.x = lastpose[ktype].tran.x;
    pos->tran.y = lastpose[ktype].tran.y;
    pos->tran.z = lastpose[ktype].tran.z;
    pos->a      = lastpose[ktype].a;
    pos->b      = lastpose[ktype].b;
    pos->c      = lastpose[ktype].c;
    pos->u      = lastpose[ktype].u;
    pos->v      = lastpose[ktype].v;
    pos->w      = lastpose[ktype].w;
} // get_lastpose()

//*********************************************************************
int kinematicsSwitchable() {return 1;}

int kinematicsSwitch(int new_kins_type)
{
    int k;
    for (k=0; k<GENHEX_KTYPES; k++) { use_lastpose[k] = 0;}
    switchkins_type = new_kins_type;
    switch (switchkins_type) {
        case 0: rtapi_print_msg(RTAPI_MSG_INFO,
                     "kinematicsSwitch:genhexkins\n");
                *switchdata->kinstype_is_0 = 1;
                *switchdata->kinstype_is_1 = 0;
                *switchdata->kinstype_is_2 = 0;
                break;
        case 1: rtapi_print_msg(RTAPI_MSG_INFO,
                     "kinematicsSwitch:Identity\n");
                *switchdata->kinstype_is_0 = 0;
                *switchdata->kinstype_is_1 = 1;
                *switchdata->kinstype_is_2 = 0;
                break;
        case 2: rtapi_print_msg(RTAPI_MSG_INFO,
                     "kinematicsSwitch:switchtype2\n");
                *switchdata->kinstype_is_0 = 0;
                *switchdata->kinstype_is_1 = 0;
                *switchdata->kinstype_is_2 = 1;
                break;
       default: rtapi_print_msg(RTAPI_MSG_ERR,
                     "kinematicsSwitch:BAD VALUE <%d>\n",
                switchkins_type);
                *switchdata->kinstype_is_1 = 0;
                *switchdata->kinstype_is_0 = 0;
                *switchdata->kinstype_is_2 = 0;
                return -1; // FAIL
    }

    use_lastpose[switchkins_type] = 1; // restarting a kins types
    return 0; // 0==> no error
} // kinematicsSwitch()

int kinematicsForward(const double *joint,
                      EmcPose * pos,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{
    int r;
    EmcPose last_genhex_pose;

    if (use_lastpose[switchkins_type]) {
        // initialize iterative forward kins (ok for identity too)
        get_lastpose(switchkins_type,pos);
        use_lastpose[switchkins_type] = 0;
    }
    switch (switchkins_type) {
       case 0: r =      genhexKinematicsForward(joint, pos, fflags, iflags);
               break;
       case 1: r =    identityKinematicsForward(joint, pos, fflags, iflags);
               break;
       case 2: r = switchtype2KinematicsForward(joint, pos, fflags, iflags);
               break;
      default: rtapi_print_msg(RTAPI_MSG_ERR,
                    "genhex-switchkins Forward BAD switchkins_type </%d>\n",
                    switchkins_type);
               return -1;
    }
    save_lastpose(switchkins_type,pos);
    if (r) return r;

    // compute pose for vismach gui
    get_lastpose(0,&last_genhex_pose);
    r = genhex_gui_forward_kins(joint,&last_genhex_pose);

    return r;
} // kinematicsForward()

int kinematicsInverse(const EmcPose * pos,
                      double *joint,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
    int r;
    switch (switchkins_type) {
       case 0:  r=      genhexKinematicsInverse(pos, joint, iflags, fflags);
                break;
       case 1:  r=    identityKinematicsInverse(pos, joint, iflags, fflags);
                break;
       case 2:  r= switchtype2KinematicsInverse(pos, joint, iflags, fflags);
                break;
       default: rtapi_print_msg(RTAPI_MSG_ERR,
                     "genhex-switchkins Inverse BAD switchkins_type </%d>\n",
                     switchkins_type);
               return -1;
    }

    return r;
} // kinematicsInverse()

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

// support 6 joints (GENHEX_MAX_JOINTS)
static char *coordinates = "XYZABC";
RTAPI_MP_STRING(coordinates, "Axes-to-joints-identity-ordering");

EXPORT_SYMBOL(kinematicsSwitchable);
EXPORT_SYMBOL(kinematicsSwitch);
EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
MODULE_LICENSE("GPL");

static int comp_id;

int rtapi_app_main(void)
{
#define DISALLOW_DUPLICATES 0
    int res = -1;
    int njoints = (int)strlen(coordinates);

    if (njoints != GENHEX_MAX_JOINTS) {
        rtapi_print_msg(RTAPI_MSG_ERR,
             "genhex-switchkins requires exactly %d letters in coordinates=%s\n",
             GENHEX_MAX_JOINTS,coordinates);
        return -1;
    }

    if (identityKinematicsSetup(coordinates,
                          GENHEX_MAX_JOINTS,
                          DISALLOW_DUPLICATES)) goto error;

    if (switchtype2KinematicsSetup(coordinates,
                                   GENHEX_MAX_JOINTS)) goto error;

    comp_id = hal_init("genhex-switchkins");
    if (comp_id < 0) return comp_id;

    if (genhex_hal_setup(comp_id)) goto error;
    switchdata = hal_malloc(sizeof(struct switchdata));
    if (!switchdata) goto error;

    if((res=hal_pin_bit_new("kinstype.is-0", HAL_OUT,
           &(switchdata->kinstype_is_0), comp_id)) < 0)
        goto error;
    if((res=hal_pin_bit_new("kinstype.is-1", HAL_OUT,
           &(switchdata->kinstype_is_1), comp_id)) < 0)
        goto error;
    if((res=hal_pin_bit_new("kinstype.is-2", HAL_OUT,
           &(switchdata->kinstype_is_2), comp_id)) < 0)
        goto error;

    switchkins_type   = 0;            // startup with default (0) type
    kinematicsSwitch(switchkins_type);

    hal_ready(comp_id);
    return 0;

  error:
    hal_exit(comp_id);
    return res;
} // rtapi_app_main()

void rtapi_app_exit(void)
{
    hal_exit(comp_id);
}
