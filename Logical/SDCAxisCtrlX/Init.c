#include <bur/plctypes.h>
#include <AsDefault.h>
#include <string.h>
#include "../Libraries/MotrContr/Lab3Settings.h"

#ifdef LAB3_SIMULATION
#include "../SDCAxisSimX/SimSettings.h"
#endif

void _INIT ProgramInit(void)
{
    gAxis_X_HW.EncIf1_Typ = ncSDC_ENC16;
    gAxis_X_HW.DrvIf_Typ = ncSDC_DRVSERVO16;
    gAxis_X_HW.DiDoIf_Typ = ncSDC_DIDO;
    strcpy((char*)gAxis_X_HW.EncIf1_Name, "gAxis_X_EncIf");
    strcpy((char*)gAxis_X_HW.DrvIf_Name, "gAxis_X_DrvIf");
    strcpy((char*)gAxis_X_HW.DiDoIf_Name, "gAxis_X_DiDoIf");

    fb_controller.k_p = LAB3_SPEED_KP;
    fb_controller.k_i = LAB3_SPEED_KI;
    fb_controller.dt = LAB3_CYCLE_SECONDS;
    fb_controller.max_abs_value = LAB3_MOTOR_VOLTAGE;
    pwm_period = LAB3_PWM_PERIOD_US;
    axis_X.pwm_value = 0;
    coil_pwm_value = 0;

#ifdef LAB3_SIMULATION
    coil_powered = 1;
    pwm_module_ok = 1;
    input_module_ok = 1;
    gAxis_X_EncIf.iEncOK = 1;
    gAxis_X_DrvIf.iDrvOK = 1;
    gAxis_X_DiDoIf.iDriveReady = 1;
    sim_motor.dt = LAB3_CYCLE_SECONDS;
    sim_motor.Tm = LAB3_SIM_MOTOR_TIME_SECONDS;
    sim_motor.ke = LAB3_MOTOR_VOLTAGE * LAB3_UNITS_PER_REV /
        (60.0f * LAB3_SPEED_FULL_SCALE);
    sim_motor.phi = LAB3_SIM_START_UNITS * LAB3_TWO_PI / LAB3_UNITS_PER_REV;
#endif
}
