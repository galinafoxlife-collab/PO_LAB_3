#include <bur/plctypes.h>
#include "MotrContr.h"

void FB_Integrator(struct FB_Integrator* inst)
{
    inst->out += inst->in * inst->dt;
}
