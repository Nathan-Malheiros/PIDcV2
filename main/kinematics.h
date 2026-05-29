#pragma once

/*
 * 3RPS parallel-manipulator inverse kinematics.
 * ----------------------------------------------
 * C port of Aaed Musa's "InverseKinematics" Arduino library
 * (see codes/Ball_Balancing.ino). Given a desired platform height and a
 * tilt normal vector, it returns the angle each motor arm must hold.
 *
 * Legs are arranged 120 deg apart:  A=0, B=1, C=2.
 *
 * Units are arbitrary but MUST be consistent across d/e/f/g/hz (use mm here).
 *   d  : base radius   — base pivot distance from the base centre
 *   e  : platform radius — ball-joint distance from the platform centre
 *   f  : arm length    — the link rigidly fixed to the motor shaft
 *   g  : rod length    — the link connecting the arm to the platform joint
 */

#define LEG_A 0
#define LEG_B 1
#define LEG_C 2

typedef struct {
    double d;   /* base radius   */
    double e;   /* platform radius */
    double f;   /* motor arm length */
    double g;   /* connecting rod length */
} machine_t;

void machine_init(machine_t *m, double d, double e, double f, double g);

/*
 * Angle (degrees) for one leg.
 *   leg : LEG_A / LEG_B / LEG_C
 *   hz  : platform centre height
 *   nx  : tilt normal X component (0 = level)
 *   ny  : tilt normal Y component (0 = level)
 *
 * nx/ny are the same small dimensionless quantities the PID emits
 * (typically clamped to about +/-0.25).
 */
double machine_theta(const machine_t *m, int leg, double hz, double nx, double ny);
