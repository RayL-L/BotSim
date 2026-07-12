package org.darbots.sim.vrbridge;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.darbots.sim.api.IHardwareMap;
import org.darbots.sim.api.IIMU;
import org.darbots.sim.api.IMotor;
import org.darbots.sim.api.IServo;

/** Wraps vr_physics-master's own HardwareMap so ported robot-code can run against it. */
public class VrPhysicsHardwareMap implements IHardwareMap {

    private final HardwareMap hardwareMap;

    public VrPhysicsHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public IMotor getMotor(String name) {
        return new VrPhysicsMotor(hardwareMap.get(DcMotorEx.class, name));
    }

    @Override
    public IServo getServo(String name) {
        return new VrPhysicsServo(hardwareMap.get(Servo.class, name));
    }

    @Override
    public IIMU getIMU(String name) {
        return new VrPhysicsIMU(hardwareMap.get(BNO055IMUImpl.class, name));
    }
}
