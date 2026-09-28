/*
 * airbrake_io.c
 *
 * See airbrake_io.h.
 */

#include "airbrake_io.h"
#include "config.h"
#include <math.h>

#if !AB_SERVO_MAPPING_VERIFIED
#pragma GCC warning "config.h: airbrake servo angle mapping is UNVERIFIED - measure it, then set AB_SERVO_MAPPING_VERIFIED to 1"
#endif

/* Above this tilt, 1/cos blows up; the brakes are inhibited long before. */
#define AB_IO_SPEED_TILT_LIMIT_DEG 80.0f
#define AB_IO_DEG_TO_RAD           0.017453292f

AB_Status ab_io_configure(float launch_elevation_msl_m)
{
    AB_FlightConfig cfg;
    cfg.mass_kg = AB_COAST_MASS_KG;
    cfg.target_apogee_agl_m = AB_TARGET_APOGEE_AGL_M;
    cfg.launch_elevation_msl_m = launch_elevation_msl_m;
    cfg.cd_scale = 1.0f;
    cfg.deploy_rate_per_s = AB_DEPLOY_RATE_PER_S;
    cfg.retract_rate_per_s = AB_RETRACT_RATE_PER_S;
    return ab_configure(&cfg);
}

bool ab_io_armed(const FusionOutput *fusion, FSM_State_t state)
{
    return AIRBRAKE_ENABLED &&
           state == FSM_STATE_BURNOUT &&
           fusion->ready &&
           fusion->in_flight &&
           !fusion->descent &&
           fusion->baro_ok &&
           fusion->tilt_deg < AB_MAX_TILT_DEG;
}

AB_TickInput ab_io_make_input(const FusionOutput *fusion, FSM_State_t state, float dt_s)
{
    /* The fusion gives vertical velocity; drag needs total airspeed.
     * Nose tilt stands in for flight path angle, which holds in coast
     * because the rocket weathercocks into its velocity. */
    float tilt_deg = fusion->tilt_deg;
    if (!(tilt_deg < AB_IO_SPEED_TILT_LIMIT_DEG)) {
        tilt_deg = AB_IO_SPEED_TILT_LIMIT_DEG;
    }

    AB_TickInput in;
    in.altitude_agl_m = fusion->altitude;
    in.velocity_up_mps = fusion->velocity;
    in.speed_mps = fabsf(fusion->velocity) / cosf(tilt_deg * AB_IO_DEG_TO_RAD);
    in.dt_s = dt_s;
    in.inhibit = ab_io_armed(fusion, state) ? 0 : 1;
    return in;
}

float ab_io_servo_angle_deg(float deployment)
{
    /* the airbrake core guarantees a finite command in [0, 1] */
    return MOTOR_ANGLE_MAX_DEG - deployment * (MOTOR_ANGLE_MAX_DEG - MOTOR_ANGLE_MIN_DEG);
}
