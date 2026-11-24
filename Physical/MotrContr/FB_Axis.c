#include <bur/plctypes.h>
#include "MotrContr.h"
#include "Lab3Settings.h"

void FB_Axis(struct FB_Axis* inst)
{
    DINT delta;
    REAL raw_speed;
    REAL voltage = inst->u;

    inst->reset_counter = inst->reset_counter_request;
    if (!inst->encoder_ok || inst->reset_counter) {
        inst->counter_valid = 0;
        inst->speed = 0.0f;
    } else if (inst->counter_valid) {
        delta = (DINT)inst->counter - (DINT)inst->last_counter;
        /* Counter01 is a signed, wrapping 16-bit encoder register. */
        if (delta > 32767)
            delta -= 65536L;
        else if (delta < -32768L)
            delta += 65536L;

        raw_speed = (REAL)delta * LAB3_UNITS_PER_COUNT / LAB3_CYCLE_SECONDS;
        inst->speed += (raw_speed - inst->speed) *
            (LAB3_CYCLE_SECONDS / (LAB3_SPEED_FILTER_SECONDS + LAB3_CYCLE_SECONDS));
    } else {
        inst->counter_valid = 1;
    }
    inst->last_counter = inst->counter;

    if (!inst->drive_enabled || !inst->encoder_ok ||
        (inst->endswitch_a_reached && voltage > 0.0f) ||
        (inst->endswitch_b_reached && voltage < 0.0f))
        voltage = 0.0f;

    if (voltage > LAB3_MOTOR_VOLTAGE)
        voltage = LAB3_MOTOR_VOLTAGE;
    else if (voltage < -LAB3_MOTOR_VOLTAGE)
        voltage = -LAB3_MOTOR_VOLTAGE;
    inst->pwm_value = (INT)(voltage * LAB3_PWM_FULL_SCALE / LAB3_MOTOR_VOLTAGE);
}
