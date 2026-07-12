package org.darbots.sim.vrbridge;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.darbots.sim.api.IMotor;

/**
 * Adapts vr_physics-master's own stub DcMotorEx (real ODE4J-backed physics, driven by
 * VirtualRobotController's OpMode runner) to the project's IMotor interface, so the
 * already-ported robot-code classes (MecanumDrive, Shooters, TeleopDrive, AutoRoute) can
 * run against it unchanged.
 */
public class VrPhysicsMotor implements IMotor {

    private final DcMotorEx motor;

    public VrPhysicsMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    @Override public void setPower(double power) { motor.setPower(power); }
    @Override public double getPower() { return motor.getPower(); }

    @Override
    public void setMode(RunMode mode) {
        motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.valueOf(mode.name()));
    }

    @Override
    public RunMode getMode() {
        return RunMode.valueOf(motor.getMode().name());
    }

    @Override public int getCurrentPosition() { return motor.getCurrentPosition(); }
    @Override public void setTargetPosition(int ticks) { motor.setTargetPosition(ticks); }
    @Override public int getTargetPosition() { return motor.getTargetPosition(); }
    @Override public boolean isBusy() { return motor.isBusy(); }

    @Override public void setVelocity(double ticksPerSecond) { motor.setVelocity(ticksPerSecond); }
    @Override public double getVelocity() { return motor.getVelocity(); }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.valueOf(behavior.name()));
    }

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.valueOf(direction.name()));
    }
}
