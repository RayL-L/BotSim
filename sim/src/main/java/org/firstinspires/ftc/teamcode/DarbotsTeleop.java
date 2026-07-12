package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.darbots.robot.teleop.TeleopDrive;
import org.darbots.sim.vrbridge.VrPhysicsHardwareMap;

/**
 * Thin bridge: constructs a VrPhysicsHardwareMap over this OpMode's real (vr_physics-backed)
 * hardwareMap, then just calls the already-ported TeleopDrive each loop tick. Package/annotation
 * required for VirtualRobotController's reflection-based OpMode discovery.
 */
@TeleOp(name = "Darbots Teleop", group = "Darbots")
public class DarbotsTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleopDrive drive = new TeleopDrive(new VrPhysicsHardwareMap(hardwareMap));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.a);
            telemetry.update();
            idle();
        }
    }
}
