package org.darbots.robot.drive;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;

import org.darbots.sim.api.AngleUnit;
import org.darbots.sim.api.IHardwareMap;
import org.darbots.sim.api.IIMU;
import org.darbots.sim.api.IMotor;
import org.darbots.config.RobotHardwareConfig;

/**
 * Ported from TwoDeadWheelLocalizer.java. Odometry math is unchanged; the original
 * RoadRunner-ftc Encoder wrappers and the raw FTC IMU are replaced with direct
 * IMotor/IIMU reads.
 */
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = -1104.0866733878854; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -2960.7737315716777; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final IMotor par, perp;

    public final IIMU imu;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;

    public TwoDeadWheelLocalizer(IHardwareMap hardwareMap, IIMU imu, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        par = hardwareMap.getMotor(RobotHardwareConfig.RF);
        perp = hardwareMap.getMotor(RobotHardwareConfig.LB);

        par.setDirection(IMotor.Direction.REVERSE);
        perp.setDirection(IMotor.Direction.REVERSE);

        this.imu = imu;

        lastParPos = par.getCurrentPosition();
        lastPerpPos = perp.getCurrentPosition();
        lastHeading = Rotation2d.exp(imu.getYaw(AngleUnit.RADIANS));

        this.inPerTick = inPerTick;
    }

    // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
    private double getHeadingVelocity() {
        double rawHeadingVel = imu.getYawAngularVelocity(AngleUnit.RADIANS);
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        return headingVelOffset + rawHeadingVel;
    }

    public Twist2dDual<Time> update() {
        int parPos = par.getCurrentPosition();
        double parVel = par.getVelocity();
        int perpPos = perp.getCurrentPosition();
        double perpVel = perp.getVelocity();

        Rotation2d heading = Rotation2d.exp(imu.getYaw(AngleUnit.RADIANS));

        int parPosDelta = parPos - lastParPos;
        int perpPosDelta = perpPos - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        double headingVel = getHeadingVelocity();

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parVel - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpVel - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPos;
        lastPerpPos = perpPos;
        lastHeading = heading;

        return twist;
    }
}
