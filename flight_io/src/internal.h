#ifndef FLIGHT_IO_INTERNAL_H
#define FLIGHT_IO_INTERNAL_H
#include "flight_common/control.h"
#include <math.h>
#include <stddef.h>

static inline bool fio_controls_valid(const flight_controls_t *c)
{
    return c != NULL && isfinite(c->aileron) && fabsf(c->aileron) <= 1.0f &&
           isfinite(c->elevator) && fabsf(c->elevator) <= 1.0f &&
           isfinite(c->rudder) && fabsf(c->rudder) <= 1.0f &&
           isfinite(c->throttle) && c->throttle >= 0.0f && c->throttle <= 1.0f;
}
#endif
