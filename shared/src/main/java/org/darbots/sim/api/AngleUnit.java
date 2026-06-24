package org.darbots.sim.api;

public enum AngleUnit {
    DEGREES, RADIANS;

    public double toRadians(double value) {
        return this == DEGREES ? Math.toRadians(value) : value;
    }

    public double toDegrees(double value) {
        return this == RADIANS ? Math.toDegrees(value) : value;
    }
}
