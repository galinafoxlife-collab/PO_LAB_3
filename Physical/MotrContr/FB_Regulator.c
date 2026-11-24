#include <bur/plctypes.h>
#include "MotrContr.h"

static REAL limitVoltage(REAL value, REAL bound)
{
    if (value > bound)
        return bound;
    if (value < -bound)
        return -bound;
    return value;
}

void FB_Regulator(struct FB_Regulator* inst)
{
    REAL proportional;
    REAL sum;

    if (!inst->enabled || inst->dt <= 0.0f || inst->max_abs_value <= 0.0f) {
        inst->integrator.in = 0.0f;
        inst->integrator.out = 0.0f;
        inst->u = 0.0f;
        return;
    }

    proportional = limitVoltage(inst->k_p * inst->e, inst->max_abs_value);
    sum = proportional + inst->integrator.out;
    inst->integrator.in = inst->k_i * inst->e;
    inst->integrator.dt = inst->dt;
    if ((sum < inst->max_abs_value || inst->integrator.in < 0.0f) &&
        (sum > -inst->max_abs_value || inst->integrator.in > 0.0f))
        FB_Integrator(&inst->integrator);

    inst->integrator.out = limitVoltage(inst->integrator.out, inst->max_abs_value);
    inst->u = limitVoltage(proportional + inst->integrator.out, inst->max_abs_value);
}
