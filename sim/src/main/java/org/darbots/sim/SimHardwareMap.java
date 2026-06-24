package org.darbots.sim;

import org.darbots.sim.api.IHardwareMap;
import org.darbots.sim.api.IIMU;
import org.darbots.sim.api.IMotor;
import org.darbots.sim.api.IServo;

import java.util.HashMap;
import java.util.Map;

public class SimHardwareMap implements IHardwareMap {

    private final Map<String, IMotor> motors = new HashMap<>();
    private final Map<String, IServo> servos = new HashMap<>();
    private final Map<String, IIMU> imus = new HashMap<>();

    public void addMotor(String name, IMotor motor) { motors.put(name, motor); }
    public void addServo(String name, IServo servo) { servos.put(name, servo); }
    public void addIMU(String name, IIMU imu) { imus.put(name, imu); }

    @Override
    public IMotor getMotor(String name) {
        IMotor m = motors.get(name);
        if (m == null) throw new IllegalArgumentException("No motor registered: " + name);
        return m;
    }

    @Override
    public IServo getServo(String name) {
        IServo s = servos.get(name);
        if (s == null) throw new IllegalArgumentException("No servo registered: " + name);
        return s;
    }

    @Override
    public IIMU getIMU(String name) {
        IIMU imu = imus.get(name);
        if (imu == null) throw new IllegalArgumentException("No IMU registered: " + name);
        return imu;
    }
}
