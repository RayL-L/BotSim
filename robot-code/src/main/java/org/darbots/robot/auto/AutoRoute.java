package org.darbots.robot.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.darbots.robot.drive.MecanumDrive;
import org.darbots.robot.subsystem.Shooters;

/**
 * Ported from autonomous/Auto.java. Route poses, timings and the shooting sequence are
 * unchanged. The original extended ScrappyAutoBase (an FTC OpMode) -- that lifecycle
 * doesn't exist here, so blue/close are constructor parameters instead of static fields,
 * and this class just builds the Action; something else (RobotSimHarness) schedules it.
 *
 * NOTE: the stopAndAdd(...) callbacks below call sleep(2000) exactly like the original
 * OpMode.sleep() calls did -- this blocks whatever thread is running the action for that
 * duration. That's the original behavior being preserved, not a mistake.
 */
public class AutoRoute {

    public final Pose2d startPose;
    public final Action route;

    public AutoRoute(MecanumDrive drive, Shooters shooters, boolean blue, boolean close) {
        // ===== START POSE =====
        if (blue && close)          startPose = new Pose2d(-50, 50, Math.toRadians(-45));
        else if (blue)              startPose = new Pose2d( -11, -61, Math.toRadians(270));
        else if (close)             startPose = new Pose2d(50,  50, Math.toRadians(-135));
        else                        startPose = new Pose2d( 11,  -61, Math.toRadians(270));

        // ===== SHOOTING POSE =====
        Vector2d shootPos;
        double shootHeading;
        if (blue && close) {
            shootPos = new Vector2d(-23.5, 23.5);
            shootHeading = Math.toRadians(-45);
        } else if (blue) {
            shootPos = new Vector2d(-11, -50);
            shootHeading = Math.toRadians(-60);
        } else if (close) {
            shootPos = new Vector2d(23.5, 23.5);
            shootHeading = Math.toRadians(-135);
        } else {
            shootPos = new Vector2d(11, -50);
            shootHeading = Math.toRadians(-120);
        }

        // ===== BALL / GRAB params =====
        double ball_x = blue ? -23.5 : 23.5;
        double y21 = 8, y22 = -15, y23 = -40;
        double grab = blue ? -29 : 29;
        double ballDir = blue ? Math.toRadians(180) : Math.toRadians(0);
        double first_ball, second_ball;

        if (close) {
            first_ball = y21; second_ball = y22;
        } else {
            first_ball = y23; second_ball = y22;
        }

        // ===== ROUTE =====
        drive.pose = startPose;
        route = drive.actionBuilder(startPose)
                //preload shooting
                .strafeToLinearHeading(shootPos, shootHeading)
                .stopAndAdd(() -> {
                    shooters.shoot();
                    sleep(2000);
                    shooters.convey();
                    sleep(2000);
                    shooters.stop();
                }) //Shooting Preload Balls


                //first line of balls
                .strafeToLinearHeading(new Vector2d(ball_x, first_ball), ballDir)
                .afterTime(0.2, shooters::startIntake)
                .afterTime(1.85, shooters::stopIntake) //picking balls
                .strafeTo(new Vector2d(ball_x + grab, first_ball))

                .strafeToLinearHeading(shootPos, shootHeading)
                .stopAndAdd(() -> {
                    shooters.shoot();
                    sleep(2000);
                    shooters.convey();
                    sleep(2000);
                    shooters.stop();
                })


                //second line of balls
                .strafeToLinearHeading(new Vector2d(ball_x, second_ball), ballDir)
                .afterTime(0.2, shooters::startIntake)//picking balls
                .afterTime(1.85, shooters::stopIntake)
                .strafeTo(new Vector2d(ball_x + grab, second_ball))

                .strafeToLinearHeading(shootPos, shootHeading)
                .stopAndAdd(() -> {
                    shooters.shoot();
                    sleep(2000);
                    shooters.convey();
                    sleep(2000);
                    shooters.stop();
                })
                .build();
    }

    /** Mirrors the blocking behavior of the original OpMode.sleep() calls used inside stopAndAdd callbacks. */
    private static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
