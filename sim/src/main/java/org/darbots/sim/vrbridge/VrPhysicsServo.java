package org.darbots.sim.vrbridge;

import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.sim.api.IServo;

public class VrPhysicsServo implements IServo {

    private final Servo servo;

    public VrPhysicsServo(Servo servo) {
        this.servo = servo;
    }

    @Override public void setPosition(double position) { servo.setPosition(position); }
    @Override public double getPosition() { return servo.getPosition(); }

    @Override
    public void setDirection(Direction direction) {
        servo.setDirection(Servo.Direction.valueOf(direction.name()));
    }
}
