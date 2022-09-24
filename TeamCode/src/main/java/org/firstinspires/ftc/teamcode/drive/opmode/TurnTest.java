package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.GFORCE_KiwiDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {
            drive.turn(Math.toRadians(ANGLE));

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading Rad.", poseEstimate.getHeading());
            telemetry.addData("heading Deg.", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
            while (opModeIsActive() && !gamepad1.y) {};

        }
    }
}
