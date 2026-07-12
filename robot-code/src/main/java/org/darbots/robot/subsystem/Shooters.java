package org.darbots.robot.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.darbots.config.RobotHardwareConfig;
import org.darbots.sim.api.IHardwareMap;
import org.darbots.sim.api.IMotor;

public class Shooters extends SubsystemBase {

    // TODO: set this correctly for YOUR motor
    // Examples:
    // goBILDA 5202/5203 (yellow jacket) = 28 ticks/rev (motor shaft)
    // REV HD Hex = 28 ticks/rev
    // If you have external encoder / different motor, this changes.
    private static final double TICKS_PER_REV = 28.0;

    // gear ratio from motor to flywheel:
    // if motor -> flywheel is 1:1, use 1.0
    // if motor gear is smaller (speed-up), this changes
    private static final double GEAR_RATIO = 1.0;

    private final IMotor shootL;
    private final IMotor shootR;

    private final IMotor Intak;
    private final IMotor Ramp;

    private double targetRpm = 0;

    public Shooters(final IHardwareMap hwMap) {
        shootL = hwMap.getMotor(RobotHardwareConfig.SHOOT_L);
        shootR = hwMap.getMotor(RobotHardwareConfig.SHOOT_R);

        Intak = hwMap.getMotor(RobotHardwareConfig.INTAKE);
        Ramp  = hwMap.getMotor(RobotHardwareConfig.RAMP);

        // Make both "forward" for shooting by setting direction
        shootL.setDirection(IMotor.Direction.FORWARD);
        shootR.setDirection(IMotor.Direction.REVERSE);

        // Enable encoder-based velocity control
        shootL.setMode(IMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootR.setMode(IMotor.RunMode.STOP_AND_RESET_ENCODER);

        shootL.setMode(IMotor.RunMode.RUN_USING_ENCODER);
        shootR.setMode(IMotor.RunMode.RUN_USING_ENCODER);

        shootL.setZeroPowerBehavior(IMotor.ZeroPowerBehavior.FLOAT);
        shootR.setZeroPowerBehavior(IMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Set both flywheels to the SAME target RPM（转速） */
    public void setShooterRpm(double rpm) {
        targetRpm = rpm;

        double ticksPerSecond = rpmToTicksPerSecond(rpm);

        // Velocity control (the SDK will regulate speed)
        shootL.setVelocity(ticksPerSecond);
        shootR.setVelocity(ticksPerSecond);
    }

    /** Your old "shoot()" becomes "spin up to a chosen RPM" */
    public void shoot() {
        // Pick a starting value; tune later (ex: 2500~4500 depending on build)
        setShooterRpm(1500);
    }

    public void stopShooter() {
        targetRpm = 0;
        shootL.setVelocity(0);
        shootR.setVelocity(0);
    }

    public void stop(){
        stopShooter();
        Ramp.setPower(0);
        Intak.setPower(0);
    }

    public void convey(){
        Ramp.setPower(-1);
        Intak.setPower(1);
    }
    public void conveyOff(){
        Ramp.setPower(0);
        Intak.setPower(0);
    }

    public void startIntake(){
        Intak.setPower(1);
        Ramp.setPower(-0.7);
    }

    public void stopIntake(){
        Intak.setPower(0);
        Ramp.setPower(0);
    }

    public void will(){
        Ramp.setPower(0.4);
    }
    public void willStop(){
        Ramp.setPower(0);
    }

    // --- Helpers / Debug ---

    private double rpmToTicksPerSecond(double rpm) {
        // motor ticks/sec = (rpm / 60) * ticksPerRev * gearRatio
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }

    /** Actual RPM from encoder velocity (use for telemetry/debug) */
    public double getLeftRpm() {
        return ticksPerSecondToRpm(shootL.getVelocity());
    }

    public double getRightRpm() {
        return ticksPerSecondToRpm(shootR.getVelocity());
    }

    private double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }
}
