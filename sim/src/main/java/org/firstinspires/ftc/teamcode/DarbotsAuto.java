package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.darbots.robot.auto.AutoRoute;
import org.darbots.robot.auto.RunAction;
import org.darbots.robot.drive.MecanumDrive;
import org.darbots.robot.subsystem.Shooters;
import org.darbots.sim.vrbridge.VrPhysicsHardwareMap;

/**
 * Thin bridge: constructs a VrPhysicsHardwareMap, builds the already-ported MecanumDrive +
 * Shooters + AutoRoute against it, and drives the CommandScheduler each loop tick until the
 * route finishes. Package/annotation required for VirtualRobotController's reflection-based
 * OpMode discovery.
 */
@Autonomous(name = "Darbots Auto", group = "Darbots")
public class DarbotsAuto extends LinearOpMode {

    public static boolean blue = true;
    public static boolean close = true;

    @Override
    public void runOpMode() throws InterruptedException {
        VrPhysicsHardwareMap hwMap = new VrPhysicsHardwareMap(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hwMap, new Pose2d(0, 0, 0));
        Shooters shooters = new Shooters(hwMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        AutoRoute route = new AutoRoute(drive, shooters, blue, close);
        Command runRoute = new RunAction(route.route);
        CommandScheduler.getInstance().schedule(runRoute);

        while (opModeIsActive() && !runRoute.isFinished()) {
            CommandScheduler.getInstance().run();
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.update();
            idle();
        }
    }
}
