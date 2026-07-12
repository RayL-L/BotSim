package org.darbots.robot.teleop;

import org.darbots.config.RobotHardwareConfig;
import org.darbots.sim.api.IHardwareMap;
import org.darbots.sim.api.IMotor;

/**
 * Drive-only port of RobotZhang.java's teleop driving (mix-drive formula, stick mapping,
 * reverse toggle). Shooter/intake/ramp control was intentionally left out of this port.
 *
 * Not tied to any particular input library (FTC Gamepad, Jamepad, keyboard, etc.) --
 * call update() once per teleop tick with whatever the current stick/button state is.
 */
public class TeleopDrive {

    private final IMotor LF, RF, LB, RB;

    private boolean driveReversed = false;
    private boolean previousReverseButton = false;

    public double speed = 0.7;

    public TeleopDrive(IHardwareMap hardwareMap) {
        LF = hardwareMap.getMotor(RobotHardwareConfig.LF);
        RF = hardwareMap.getMotor(RobotHardwareConfig.RF);
        LB = hardwareMap.getMotor(RobotHardwareConfig.LB);
        RB = hardwareMap.getMotor(RobotHardwareConfig.RB);

        LF.setZeroPowerBehavior(IMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(IMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(IMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(IMotor.ZeroPowerBehavior.BRAKE);

        LF.setDirection(IMotor.Direction.REVERSE);
        LB.setDirection(IMotor.Direction.REVERSE);
        RF.setDirection(IMotor.Direction.REVERSE);
        RB.setDirection(IMotor.Direction.REVERSE);
    }

    /**
     * Call once per teleop tick with the current driver-1 stick/button state.
     * Mirrors RobotZhang.java's drive handling exactly (stick mapping, reverse toggle, mix-drive formula).
     */
    public void update(double leftStickX, double leftStickY, double rightStickX, boolean reverseButtonPressed) {
        if (reverseButtonPressed && !previousReverseButton) {
            driveReversed = !driveReversed;
        }
        previousReverseButton = reverseButtonPressed;

        double drive = leftStickY;
        double strafe = leftStickX * -1;
        double rotate = rightStickX * -1;

        if (driveReversed) {
            drive = -drive;
            strafe = -strafe;
        }

        setDriving(drive, strafe, rotate, speed);
    }

    private void setDriving(double drive, double strafe, double rotate, double speed) {
        double leftFrontPower  = drive + strafe + rotate;
        double rightFrontPower = drive - strafe - rotate;
        double leftBackPower   = drive - strafe + rotate;
        double rightBackPower  = drive + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        LF.setPower(leftFrontPower * speed);
        RF.setPower(rightFrontPower * speed);
        LB.setPower(leftBackPower * speed);
        RB.setPower(rightBackPower * speed);
    }
}
