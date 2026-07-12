package org.darbots.config;

/**
 * Single source of truth for hardware device names. Referenced identically by the
 * real-robot code (via IHardwareMap) and by the simulator's SimHardwareMap registration,
 * so both sides always agree on what a given device is called.
 */
public final class RobotHardwareConfig {
    private RobotHardwareConfig() {}

    // Drivetrain
    public static final String LF = "LF";
    public static final String RF = "RF";
    public static final String LB = "LB";
    public static final String RB = "RB";
    public static final String IMU = "imu";

    // Shooter subsystem
    public static final String SHOOT_L = "shootL";
    public static final String SHOOT_R = "shootR";
    public static final String INTAKE = "Intake";
    public static final String RAMP = "Ramp";
}
