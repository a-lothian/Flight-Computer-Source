/*
 * airbrake_io.h
 *
 * Glue between the flight computer and the airbrake core (airbrake_api.h):
 * builds each tick's input from the sensor fusion and FSM, decides when the
 * brakes are allowed to move, and maps the deployment command to a servo
 * angle. Kept out of main.c so it can be unit tested on a PC.
 *
 * Contract with the airbrake core (see airbrake_api.h): call ab_tick() every
 * loop, including while inhibited, or its model of where the servo is goes
 * stale.
 */

#ifndef INC_AIRBRAKE_IO_H_
#define INC_AIRBRAKE_IO_H_

#include "airbrake_api.h"
#include "fsm.h"
#include "sensor_fusion.h"

/* Configures the airbrake core with the values in config.h.
 * launch_elevation_msl_m is the pad's pressure altitude, which the ISA
 * density model needs (the fusion only knows altitude above the pad). */
AB_Status ab_io_configure(float launch_elevation_msl_m);

/* True only while the brakes are allowed to move: in coast (FSM BURNOUT,
 * which runs from burnout to apogee), fusion calibrated and in flight,
 * before the fusion's own apogee, baro healthy, and nose within
 * AB_MAX_TILT_DEG of vertical. */
bool ab_io_armed(const FusionOutput *fusion, FSM_State_t state);

/* This loop's input to ab_tick(). */
AB_TickInput ab_io_make_input(const FusionOutput *fusion, FSM_State_t state, float dt_s);

/* Deployment fraction (0..1) to servo angle in degrees. */
float ab_io_servo_angle_deg(float deployment);

#endif /* INC_AIRBRAKE_IO_H_ */
