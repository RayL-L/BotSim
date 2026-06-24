package org.darbots.sim;

import org.darbots.sim.api.IMotor;

public class SimMotor implements IMotor {

    private double power = 0;
    private int currentPosition = 0;
    private int targetPosition = 0;
    private double velocity = 0;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
    private Direction direction = Direction.FORWARD;

    @Override public void setPower(double power) { this.power = power; }
    @Override public double getPower() { return power; }

    @Override public void setMode(RunMode mode) { this.mode = mode; }
    @Override public RunMode getMode() { return mode; }

    @Override public int getCurrentPosition() { return currentPosition; }
    @Override public void setTargetPosition(int ticks) { this.targetPosition = ticks; }
    @Override public int getTargetPosition() { return targetPosition; }
    @Override public boolean isBusy() { return false; }

    @Override public void setVelocity(double ticksPerSecond) { this.velocity = ticksPerSecond; }
    @Override public double getVelocity() { return velocity; }

    @Override public void setZeroPowerBehavior(ZeroPowerBehavior behavior) { this.zeroPowerBehavior = behavior; }
    @Override public void setDirection(Direction direction) { this.direction = direction; }

    public void updatePosition(int delta) { currentPosition += delta; }
}
