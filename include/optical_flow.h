#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H
#include "types.h"

void optical_flow_init();
void optical_flow_start_task();
void optical_flow_update(DroneState *drone);
void optical_flow_set_scale(float scale);
void optical_flow_reset_position();

#endif
