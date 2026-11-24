#include <bur/plctypes.h>
#include <AsDefault.h>

void _EXIT ProgramExit(void)
{
    axis_X.drive_enabled = 0;
    axis_X.pwm_value = 0;
    axis_X.reset_error = 0;
    axis_X.reset_counter = 0;
    coil_pwm_value = 0;
    coil_clear_error = 0;
    gAxis_X_DrvIf.iStatusEnable = 0;
    gAxis_X_DiDoIf.iDriveReady = 0;
}
