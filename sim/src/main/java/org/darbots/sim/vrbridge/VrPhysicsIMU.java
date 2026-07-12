package org.darbots.sim.vrbridge;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.darbots.sim.api.AngleUnit;
import org.darbots.sim.api.IIMU;

/**
 * Adapts vr_physics-master's BNO055IMUImpl (older Orientation-based API, no built-in
 * angular-velocity reading) to IIMU's simpler getYaw/getYawAngularVelocity shape.
 * Angular velocity is derived by finite-differencing yaw between calls -- there's no
 * direct gyro-rate reading available on this stub.
 */
public class VrPhysicsIMU implements IIMU {

    private final BNO055IMUImpl imu;

    private double lastYawRadians = 0;
    private long lastNanos = -1;
    private double yawRateRadiansPerSec = 0;

    public VrPhysicsIMU(BNO055IMUImpl imu) {
        this.imu = imu;
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
    }

    @Override
    public double getYaw(AngleUnit unit) {
        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit ftcUnit =
                unit == AngleUnit.DEGREES
                        ? org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES
                        : org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

        Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, ftcUnit);
        double yaw = o.firstAngle;

        trackYawRate(unit == AngleUnit.DEGREES ? Math.toRadians(yaw) : yaw);
        return yaw;
    }

    @Override
    public double getYawAngularVelocity(AngleUnit unit) {
        return unit == AngleUnit.DEGREES ? Math.toDegrees(yawRateRadiansPerSec) : yawRateRadiansPerSec;
    }

    private void trackYawRate(double yawRadians) {
        long now = System.nanoTime();
        if (lastNanos > 0) {
            double dt = (now - lastNanos) / 1_000_000_000.0;
            if (dt > 0) {
                double delta = yawRadians - lastYawRadians;
                if (delta > Math.PI) delta -= 2 * Math.PI;
                else if (delta < -Math.PI) delta += 2 * Math.PI;
                yawRateRadiansPerSec = delta / dt;
            }
        }
        lastYawRadians = yawRadians;
        lastNanos = now;
    }
}
