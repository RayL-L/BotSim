package org.darbots.sim.api;

public interface IServo {

    enum Direction {
        FORWARD, REVERSE
    }

    void setPosition(double position);
    double getPosition();
    void setDirection(Direction direction);
}
