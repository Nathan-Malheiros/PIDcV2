#include "kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SQRT3 1.7320508075688772

void machine_init(machine_t *m, double d, double e, double f, double g)
{
    m->d = d;
    m->e = e;
    m->f = f;
    m->g = g;
}

/*
 * Faithful port of Machine::theta from Aaed Musa's InverseKinematics library.
 * The closed-form expressions place each platform ball-joint for the requested
 * tilt, then solve the arm/rod triangle for the motor angle.
 */
double machine_theta(const machine_t *m, int leg, double hz, double nx, double ny)
{
    const double d = m->d, e = m->e, f = m->f, g = m->g;

    /* normalise the tilt normal vector (nx, ny, 1) */
    double nmag = sqrt(nx * nx + ny * ny + 1.0);
    nx /= nmag;
    ny /= nmag;
    double nz = 1.0 / nmag;

    double x, y, z, mag, angle = 0.0;

    switch (leg) {
    case LEG_A:
        y = d + (e / 2.0) * (1.0 - (nx * nx + 3.0 * nz * nz + 3.0 * nz) /
                (nz + 1.0 - nx * nx +
                 (pow(nx, 4) - 3.0 * nx * nx * ny * ny) /
                 ((nz + 1.0) * (nz + 1.0 - nx * nx))));
        z = hz + e * ny;
        mag = sqrt(y * y + z * z);
        angle = acos(y / mag) +
                acos((mag * mag + f * f - g * g) / (2.0 * mag * f));
        break;

    case LEG_B:
        x = (SQRT3 / 2.0) * (e * (1.0 - (nx * nx + SQRT3 * nx * ny) / (nz + 1.0)) - d);
        y = x / SQRT3;
        z = hz - (e / 2.0) * (SQRT3 * nx + ny);
        mag = sqrt(x * x + y * y + z * z);
        angle = acos((SQRT3 * x + y) / (-2.0 * mag)) +
                acos((mag * mag + f * f - g * g) / (2.0 * mag * f));
        break;

    case LEG_C:
        x = (SQRT3 / 2.0) * (d - e * (1.0 - (nx * nx - SQRT3 * nx * ny) / (nz + 1.0)));
        y = -x / SQRT3;
        z = hz + (e / 2.0) * (SQRT3 * nx - ny);
        mag = sqrt(x * x + y * y + z * z);
        angle = acos((-SQRT3 * x + y) / (-2.0 * mag)) +
                acos((mag * mag + f * f - g * g) / (2.0 * mag * f));
        break;
    }

    return angle * (180.0 / M_PI);
}
