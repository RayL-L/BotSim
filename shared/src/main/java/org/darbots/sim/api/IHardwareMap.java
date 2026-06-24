package org.darbots.sim.api;

public interface IHardwareMap {

    IMotor getMotor(String name);
    IServo getServo(String name);
    IIMU getIMU(String name);
}
