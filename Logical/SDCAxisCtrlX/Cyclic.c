#include <bur/plctypes.h>
#include <AsDefault.h>
#include <stddef.h>
#include "../Libraries/MotrContr/Lab3Settings.h"

#ifdef LAB3_SIMULATION
#include "../SDCAxisSimX/SimSettings.h"
#endif

enum LabState {
    labDisabled = 0,
    labReady,
    labEnabled,
    labHoming,
    labReferencedOff,
    labFault
};

static SINT nextLifeCount(SINT count)
{
    return count == 127 ? -128 : (SINT)(count + 1);
}

static void publishSdcInputs(BOOL module_healthy)
{
    gAxis_X_EncIf.iLifeCnt = nextLifeCount(gAxis_X_EncIf.iLifeCnt);
    gAxis_X_EncIf.iActTime = (INT)AsIOTimeCyclicStart();
    gAxis_X_EncIf.iActPos = axis_X.counter;
    gAxis_X_EncIf.iEncOK = axis_X.encoder_ok;

    gAxis_X_DrvIf.iLifeCnt = nextLifeCount(gAxis_X_DrvIf.iLifeCnt);
    gAxis_X_DrvIf.iDrvOK = module_healthy;
    gAxis_X_DrvIf.iStatusEnable = axis_X.drive_enabled;

    gAxis_X_DiDoIf.iLifeCntDriveEnable = nextLifeCount(gAxis_X_DiDoIf.iLifeCntDriveEnable);
    gAxis_X_DiDoIf.iLifeCntDriveReady = nextLifeCount(gAxis_X_DiDoIf.iLifeCntDriveReady);
    gAxis_X_DiDoIf.iLifeCntPosHwEnd = nextLifeCount(gAxis_X_DiDoIf.iLifeCntPosHwEnd);
    gAxis_X_DiDoIf.iLifeCntNegHwEnd = nextLifeCount(gAxis_X_DiDoIf.iLifeCntNegHwEnd);
    gAxis_X_DiDoIf.iLifeCntReference = nextLifeCount(gAxis_X_DiDoIf.iLifeCntReference);
    gAxis_X_DiDoIf.iDriveReady = module_healthy;
    gAxis_X_DiDoIf.iPosHwEnd = axis_X.endswitch_a_reached;
    gAxis_X_DiDoIf.iNegHwEnd = axis_X.endswitch_b_reached;
}

static ACP10AXIS_typ* findAxis(void)
{
    if (nc_axis_address == 0) {
        nc_access_status = ncaccess(ncACP10MAN, "gAxis_X", &nc_axis_address);
        if (nc_access_status != ncOK)
            nc_axis_address = 0;
    }
    return (ACP10AXIS_typ*)(size_t)nc_axis_address;
}

static void requestSwitchOff(BOOL active)
{
    if (!active || (!home_lockout && !fault_lockout)) {
        switch_off_wait = 0;
    } else if (switch_off_wait > 0) {
        --switch_off_wait;
    } else if (nc_axis_address != 0) {
        nc_switch_off_status = ncaction(nc_axis_address, ncCONTROLLER, ncSWITCH_OFF);
        /* Wait for the NC manager to complete or retry a busy request. */
        switch_off_wait = 49;
    }
}

#ifdef LAB3_SIMULATION
static void updateSimulatedInputs(void)
{
    REAL position;
    REAL counts;
    INT sampled_counter;

    sim_motor.u = coil_pwm_value != 0 ?
        (REAL)axis_X.pwm_value * LAB3_MOTOR_VOLTAGE / LAB3_PWM_FULL_SCALE : 0.0f;
    FB_Motor(&sim_motor);
    position = sim_motor.phi * LAB3_UNITS_PER_REV / LAB3_TWO_PI;
    if (position < -LAB3_SIM_OVERTRAVEL || position > LAB3_SIM_TRAVEL + LAB3_SIM_OVERTRAVEL) {
        position = position < 0.0f ? -LAB3_SIM_OVERTRAVEL : LAB3_SIM_TRAVEL + LAB3_SIM_OVERTRAVEL;
        sim_motor.phi = position * LAB3_TWO_PI / LAB3_UNITS_PER_REV;
        sim_motor.w = 0.0f;
    }

    counts = position / LAB3_UNITS_PER_COUNT;
    sampled_counter = (INT)(counts < 0.0f ? counts - 0.5f : counts + 0.5f);
    if (axis_X.reset_counter)
        sim_encoder_origin = sampled_counter;
    axis_X.counter = sampled_counter - sim_encoder_origin;

    axis_X.endswitch_b_reached = axis_X.endswitch_b_reached ?
        position < LAB3_SIM_SWITCH_HYSTERESIS : position <= 0.0f;
    axis_X.endswitch_a_reached = axis_X.endswitch_a_reached ?
        position > LAB3_SIM_TRAVEL - LAB3_SIM_SWITCH_HYSTERESIS : position >= LAB3_SIM_TRAVEL;
    pwm_module_ok = 1;
    input_module_ok = 1;
    motor_pwm_error = sim_fault;
}
#endif

void _CYCLIC ProgramCyclic(void)
{
    ACP10AXIS_typ* nc_axis;
    BOOL drive_requested;
    BOOL homing;
    BOOL active;
    BOOL module_healthy;
    BOOL coil_on;
    BOOL direction_blocked;

#ifdef LAB3_SIMULATION
    updateSimulatedInputs();
#endif

    nc_axis = findAxis();
    controller_on = nc_axis != 0 && nc_axis->controller.status == ncON;
    referenced = nc_axis != 0 && nc_axis->move.homing.status.ok == ncTRUE;
    homing = nc_axis != 0 && nc_axis->move.mode == ncHOMING;
    drive_requested = gAxis_X_DiDoIf.oDriveEnable != 0;
    active = drive_requested || controller_on;

    if (!coil_powered && !active) {
        home_lockout = 0;
        fault_lockout = 0;
    }
    if (nc_axis != 0) {
        if (nc_state_seen && referenced && (!previous_referenced || (previous_homing && !homing)))
            home_lockout = 1;
        previous_referenced = referenced;
        previous_homing = homing;
        nc_state_seen = 1;
    }

    module_healthy = pwm_module_ok && input_module_ok &&
        !motor_pwm_error && !coil_pwm_error && !under_voltage && !over_voltage &&
        !over_temperature && !(axis_X.endswitch_a_reached && axis_X.endswitch_b_reached);
    if (active && (!module_healthy || !coil_powered))
        fault_lockout = 1;

    axis_X.encoder_ok = pwm_module_ok;
    coil_on = nc_axis != 0 && coil_powered && module_healthy && !home_lockout && !fault_lockout;
    coil_pwm_value = coil_on ? LAB3_PWM_FULL_SCALE : 0;
    axis_X.drive_enabled = coil_on && drive_requested;
    axis_X.reset_error = pwm_module_ok && !active && reset_error_request && !previous_reset_error;
    coil_clear_error = axis_X.reset_error;
    axis_X.reset_counter_request = pwm_module_ok && !active && !coil_powered && !referenced &&
        reset_counter_request && !previous_reset_counter;
    previous_reset_error = reset_error_request;
    previous_reset_counter = reset_counter_request;

    velocity_command = (REAL)gAxis_X_DrvIf.oSetPos * LAB3_SPEED_FULL_SCALE / LAB3_PWM_FULL_SCALE;
    direction_blocked = (axis_X.endswitch_a_reached && velocity_command > 0.0f) ||
        (axis_X.endswitch_b_reached && velocity_command < 0.0f);
    /* The speed input is the estimate from the preceding 2 ms sample. */
    fb_controller.e = velocity_command - axis_X.speed;
    fb_controller.enabled = axis_X.drive_enabled && !direction_blocked;
    FB_Regulator(&fb_controller);
    axis_X.u = fb_controller.u;
    FB_Axis(&axis_X);
    publishSdcInputs(module_healthy);
    requestSwitchOff(active);

    if (fault_lockout || !module_healthy)
        lab_state = labFault;
    else if (home_lockout)
        lab_state = labReferencedOff;
    else if (axis_X.drive_enabled)
        lab_state = homing ? labHoming : labEnabled;
    else
        lab_state = coil_on ? labReady : labDisabled;
}
