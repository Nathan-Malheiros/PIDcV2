#pragma once

/*
 * Ball-balancing control loop.
 *
 * Initialises the kinematics, dual PID and stepper engine, homes the
 * platform to level, then runs the 50 Hz loop forever:
 *
 *   touch_read() -> PID(X), PID(Y) -> tilt normal (nx, ny)
 *                -> machine_theta() per leg -> step targets.
 *
 * Call once from app_main(); it does not return.
 */
void control_run(void);
