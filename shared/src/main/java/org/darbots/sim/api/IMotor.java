package org.darbots.sim.api;

public interface IMotor {

    enum RunMode {
        RUN_TO_POSITION,
        RUN_USING_ENCODER,
        RUN_WITHOUT_ENCODER,
        STOP_AND_RESET_ENCODER
    }

    enum ZeroPowerBehavior {
        BRAKE, FLOAT
    }

    enum Direction {
        FORWARD, REVERSE
    }

    void setPower(double power);
    double getPower();

    void setMode(RunMode mode);
    RunMode getMode();

    int getCurrentPosition();
    void setTargetPosition(int ticks);
    int getTargetPosition();
    boolean isBusy();

    void setVelocity(double ticksPerSecond);
    double getVelocity();

    void setZeroPowerBehavior(ZeroPowerBehavior behavior);
    void setDirection(Direction direction);
}
