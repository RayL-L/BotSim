package org.darbots.sim;

import org.darbots.sim.api.AngleUnit;
import org.darbots.sim.api.IIMU;

public class SimIMU implements IIMU {

    private double yawRadians = 0;
    private double yawRateRadiansPerSec = 0;

    @Override
    public double getYaw(AngleUnit unit) {
        return unit == AngleUnit.DEGREES ? Math.toDegrees(yawRadians) : yawRadians;
    }

    @Override
    public double getYawAngularVelocity(AngleUnit unit) {
        return unit == AngleUnit.DEGREES ? Math.toDegrees(yawRateRadiansPerSec) : yawRateRadiansPerSec;
    }

    public void setYaw(double radians) { this.yawRadians = radians; }
    public void setYawRate(double radiansPerSec) { this.yawRateRadiansPerSec = radiansPerSec; }
}
