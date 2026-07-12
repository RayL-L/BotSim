package org.darbots.sim;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.darbots.config.RobotHardwareConfig;
import org.darbots.robot.auto.AutoRoute;
import org.darbots.robot.auto.RunAction;
import org.darbots.robot.drive.MecanumDrive;
import org.darbots.robot.subsystem.Shooters;
import org.darbots.robot.teleop.TeleopDrive;

/**
 * Wires the ported robot-code classes (MecanumDrive, Shooters, TeleopDrive, AutoRoute)
 * to a SimHardwareMap, using the shared RobotHardwareConfig device names on both sides.
 *
 * This proves the whole chain compiles and runs end-to-end. It does NOT drive real
 * physics (SimMotor is still a bare stub -- ODE4J integration is a separate, not-yet-built
 * piece of work) and it does NOT read real gamepad/keyboard input -- that wiring, plus
 * an interactive JavaFX game loop, is future work.
 */
public class RobotSimHarness {

    public final SimHardwareMap hardwareMap = new SimHardwareMap();
    public final MecanumDrive drive;
    public final Shooters shooters;
    public final TeleopDrive teleopDrive;

    public RobotSimHarness() {
        hardwareMap.addMotor(RobotHardwareConfig.LF, new SimMotor());
        hardwareMap.addMotor(RobotHardwareConfig.RF, new SimMotor());
        hardwareMap.addMotor(RobotHardwareConfig.LB, new SimMotor());
        hardwareMap.addMotor(RobotHardwareConfig.RB, new SimMotor());
        hardwareMap.addMotor(RobotHardwareConfig.SHOOT_L, new SimMotor());
        hardwareMap.addMotor(RobotHardwareConfig.SHOOT_R, new SimMotor());
        hardwareMap.addMotor(RobotHardwareConfig.INTAKE, new SimMotor());
        hardwareMap.addMotor(RobotHardwareConfig.RAMP, new SimMotor());
        hardwareMap.addIMU(RobotHardwareConfig.IMU, new SimIMU());

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        shooters = new Shooters(hardwareMap);
        teleopDrive = new TeleopDrive(hardwareMap);
    }

    /** Builds the fixed autonomous route (Auto.java-equivalent) and schedules it to run. */
    public void scheduleAutoRoute(boolean blue, boolean close) {
        AutoRoute autoRoute = new AutoRoute(drive, shooters, blue, close);
        CommandScheduler.getInstance().schedule(new RunAction(autoRoute.route));
    }

    /** Advances the command scheduler by one tick. Call every simulator frame. */
    public void tick() {
        CommandScheduler.getInstance().run();
    }
}
