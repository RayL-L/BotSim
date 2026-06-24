package org.darbots.sim;

import org.darbots.sim.api.IServo;

public class SimServo implements IServo {

    private double position = 0;
    private Direction direction = Direction.FORWARD;

    @Override public void setPosition(double position) { this.position = position; }
    @Override public double getPosition() { return position; }
    @Override public void setDirection(Direction direction) { this.direction = direction; }
}
