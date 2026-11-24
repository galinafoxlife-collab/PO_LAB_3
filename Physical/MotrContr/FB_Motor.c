#include <bur/plctypes.h>
#include "MotrContr.h"
#include "Lab3Settings.h"

void FB_Motor(struct FB_Motor* inst)
{
    if (inst->ke <= 0.0f || inst->Tm <= 0.0f || inst->dt <= 0.0f)
        return;

    inst->w += (inst->u / inst->ke - inst->w) * inst->dt / inst->Tm;
    inst->phi += inst->w * (LAB3_TWO_PI / 60.0f) * inst->dt;
}
