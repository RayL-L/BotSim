package org.darbots.sim.api;

public interface IIMU {

    /** Yaw angle around the robot's vertical axis (heading). */
    double getYaw(AngleUnit unit);

    /** Angular velocity around the vertical axis (used by TwoDeadWheelLocalizer). */
    double getYawAngularVelocity(AngleUnit unit);
}
