/**
 * @file    attitude.c
 * @brief   Accelerometer tilt, gyro integration and complementary filter.
 *          See attitude.h for the background.
 *
 * Quaternion conventions: q = (w, x, y, z), unit length, rotates body-frame
 * vectors into the earth frame (z = up). Composition q_a * q_b applies q_b
 * first. The 3x3 matrix of q is written out where needed; only its bottom
 * row (what points up) and first column are ever used.
 */
#include "attitude.h"
#include <math.h>

/** 1 g in m/s^2. The accelerometer passed to attitude_update() must be in
 *  m/s^2 for the "is it reading 1 g?" check to work. */
#define G_ATT 9.80665f

/* ---- Complementary filter tuning ---------------------------------- */

/**
 * Time constant of the accelerometer correction, seconds.
 * Roughly: how long the accelerometer takes to pull out a drift error.
 *   Bigger  -> smoother angles, trusts the gyro for longer.
 *   Smaller -> corrects drift faster, but lets more accelerometer noise
 *              (vibration, handling) into the angles.
 * At 50 Hz and 1 s this is equivalent to the classic "alpha = 0.98".
 */
#ifndef ATT_TAU_S
#define ATT_TAU_S        1.0f
#endif

/**
 * Only trust the accelerometer when |a| is within this fraction of 1 g.
 * 0.1 means between 0.9 g and 1.1 g. Anything outside means something
 * other than gravity is acting (thrust, drag, a knock).
 */
#ifndef ATT_ACCEL_BAND
#define ATT_ACCEL_BAND   0.1f
#endif

/* -------------------------------------------------------------------- */
/** Hamilton product a * b (apply b, then a). */
static Quat_t quat_mul(const Quat_t *a, const Quat_t *b)
{
    Quat_t r;
    r.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    r.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    r.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    r.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
    return r;
}

/**
 * Scale back to unit length. Rounding in every multiply slowly pushes a
 * quaternion off unit length, which would turn it into a rotation plus a
 * scaling. Also keeps w >= 0 (q and -q are the same rotation).
 */
static Quat_t quat_normalise(Quat_t q)
{
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-6f) {                 /* can't happen from valid input;   */
        Quat_t id = { 1.0f, 0.0f, 0.0f, 0.0f };   /* never return NaN    */
        return id;
    }
    float s = (q.w < 0.0f) ? -1.0f / n : 1.0f / n;
    q.w *= s; q.x *= s; q.y *= s; q.z *= s;
    return q;
}

/**
 * Quaternion for a rotation of |v| radians about the direction of v.
 *
 * Exact: (cos(a/2), sin(a/2) * v/|v|). For tiny angles sin(a/2)/a is
 * replaced by its series so there's no 0/0.
 */
static Quat_t quat_from_rotvec(float vx, float vy, float vz)
{
    float a2 = vx * vx + vy * vy + vz * vz;
    float a  = sqrtf(a2);
    float s;                          /* sin(a/2) / a */
    Quat_t q;
    if (a < 1e-4f) {
        s   = 0.5f - a2 / 48.0f;
        q.w = 1.0f - a2 / 8.0f;
    } else {
        s   = sinf(0.5f * a) / a;
        q.w = cosf(0.5f * a);
    }
    q.x = vx * s;
    q.y = vy * s;
    q.z = vz * s;
    return q;
}

/* -------------------------------------------------------------------- */
/**
 * Lean from gravity.
 *
 * phi = atan2(ay, az):
 *   upright, all of gravity is on z (ay = 0) -> phi = 0.
 *   leaning 30 deg about x, gravity splits sin30 onto y, cos30 onto z
 *   -> phi = atan2(0.5, 0.866) = 30 deg.
 *   atan2 (not atan(ay/az)) keeps the sign of both inputs, so left and
 *   right leans are distinguished, and az = 0 doesn't divide by zero.
 *
 * theta = atan2(-ax, sqrt(ay^2 + az^2)):
 *   same idea for the other axis. sqrt(ay^2 + az^2) collects all gravity
 *   that is NOT on x, which keeps theta correct when phi is non-zero (the
 *   ZYX Euler order). The minus sign matches the right-hand rule: a
 *   positive rotation about y makes ax read negative.
 *
 * The quaternion of ZYX angles (phi, theta, psi = 0) is qy(theta) * qx(phi).
 */
Quat_t attitude_from_accel(float ax, float ay, float az)
{
    float phi   = atan2f(ay, az);
    float theta = atan2f(-ax, sqrtf(ay * ay + az * az));
    float cr = cosf(0.5f * phi),   sr = sinf(0.5f * phi);
    float cp = cosf(0.5f * theta), sp = sinf(0.5f * theta);
    Quat_t q = { cr * cp, sr * cp, cr * sp, -sr * sp };
    return quat_normalise(q);
}

/* -------------------------------------------------------------------- */
/**
 * Bottom row of the rotation matrix: the earth-vertical component of each
 * body axis.
 */
void attitude_up(const Quat_t *q, float up[3])
{
    up[0] = 2.0f * (q->x * q->z - q->w * q->y);
    up[1] = 2.0f * (q->y * q->z + q->w * q->x);
    up[2] = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
}

/* -------------------------------------------------------------------- */
/**
 * ZYX Euler angles from the rotation matrix R of q:
 *   phi   = atan2(R21, R22)
 *   theta = asin(-R20)
 *   psi   = atan2(R10, R00)
 * The same angles the previous Euler-based filter reported, so pitch/yaw/
 * roll keep their meaning. Near theta = +/-90 deg phi and psi become
 * ill-defined (gimbal lock) - that only affects these reported numbers,
 * never the filter itself.
 */
Euler_t attitude_euler(const Quat_t *q)
{
    float up[3];
    attitude_up(q, up);
    float r00 = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    float r10 = 2.0f * (q->x * q->y + q->w * q->z);

    float s = -up[0];
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    Euler_t e;
    e.phi   = atan2f(up[1], up[2]);
    e.theta = asinf(s);
    e.psi   = atan2f(r10, r00);
    return e;
}

/* -------------------------------------------------------------------- */
/**
 * Angle between the nose and vertical: the nose's vertical component is
 * up[2] = cos(tilt).
 */
float attitude_tilt(const Quat_t *q)
{
    float up[3];
    attitude_up(q, up);
    float c = up[2];

    /* Rounding can give e.g. 1.0000001 when perfectly upright; acosf of
     * anything outside [-1, 1] returns NaN, which would poison the output. */
    if (c >  1.0f) c =  1.0f;
    if (c < -1.0f) c = -1.0f;

    return acosf(c);
}

/* -------------------------------------------------------------------- */
/**
 * Gyro integration.
 *
 * The gyro reads rotation about the body's own axes, so the step rotation
 * is applied on the body side: q_new = q * dq, dq = rotation by w*dt.
 *
 * Treating the rate as constant over the step is the only approximation;
 * with 50 Hz point-sampled gyro and ~940 deg/s spin it stayed within ~1-3
 * deg of the Blue Raven's own 500 Hz estimate all the way to apogee.
 */
Quat_t attitude_integrate_gyro(const Quat_t *q,
                               float gx, float gy, float gz, float dt)
{
    Quat_t dq = quat_from_rotvec(gx * dt, gy * dt, gz * dt);
    return quat_normalise(quat_mul(q, &dq));
}

/* -------------------------------------------------------------------- */
/**
 * Complementary filter (see attitude.h for the full explanation).
 *
 * Accelerometer correction, in the earth frame:
 *   m = where the measured gravity direction points in the earth frame,
 *       according to the current attitude. If the attitude were perfect,
 *       m would be straight up (0, 0, 1).
 *   The rotation that takes m to straight up is about the horizontal axis
 *   m x (0,0,1) = (my, -mx, 0), by the angle between them.
 *   A fraction k = dt / (tau + dt) of that rotation is applied. At 50 Hz
 *   with tau = 1 s, k ~ 0.02: 2% of the gap closed per loop.
 * Because the correction axis is always horizontal, it can never change
 * spin about the vertical - gravity says nothing about that.
 */
Quat_t attitude_update(const Quat_t *q,
                       float ax, float ay, float az,
                       float gx, float gy, float gz, float dt,
                       bool use_accel)
{
    /* 1. The gyro does the main work, always */
    Quat_t out = attitude_integrate_gyro(q, gx, gy, gz, dt);

    /* In flight the accelerometer never measures gravity alone (thrust,
     * drag), so it must not correct the attitude at all. */
    if (!use_accel) {
        return out;
    }

    /* 2. Is the accelerometer seeing only gravity right now? */
    float a_mag = sqrtf(ax * ax + ay * ay + az * az);
    if (fabsf(a_mag - G_ATT) > ATT_ACCEL_BAND * G_ATT) {
        return out;        /* a knock or handling - skip this sample */
    }

    /* 3. Measured "up" in the earth frame: m = R * a / |a| (rows of R
     *    written out from the quaternion). */
    float bx = ax / a_mag, by = ay / a_mag, bz = az / a_mag;
    float w = out.w, x = out.x, y = out.y, z = out.z;
    float mx = (1.0f - 2.0f * (y * y + z * z)) * bx
             + 2.0f * (x * y - w * z) * by
             + 2.0f * (x * z + w * y) * bz;
    float my = 2.0f * (x * y + w * z) * bx
             + (1.0f - 2.0f * (x * x + z * z)) * by
             + 2.0f * (y * z - w * x) * bz;
    float mz = 2.0f * (x * z - w * y) * bx
             + 2.0f * (y * z + w * x) * by
             + (1.0f - 2.0f * (x * x + y * y)) * bz;

    /* 4. Rotate a fraction k of the way from m to vertical */
    float ex = my, ey = -mx;                  /* m x z_up, horizontal */
    float e_len = sqrtf(ex * ex + ey * ey);   /* = sin(angle)         */
    if (e_len < 1e-7f) {
        return out;        /* already exactly upright (or exactly inverted,
                              where no single correction axis exists) */
    }
    float angle = atan2f(e_len, mz);
    float k = dt / (ATT_TAU_S + dt);
    float s = k * angle / e_len;
    Quat_t corr = quat_from_rotvec(ex * s, ey * s, 0.0f);

    /* earth-frame rotation, so it is applied on the left */
    return quat_normalise(quat_mul(&corr, &out));
}
