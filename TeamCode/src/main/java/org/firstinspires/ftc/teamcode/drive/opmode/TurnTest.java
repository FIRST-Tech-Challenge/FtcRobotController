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
    Pose2d poseEstimate;

    @Override
    public void runOpMode() throws InterruptedException {
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.turn(Math.toRadians(45));
        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("heading Deg.", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
        //while(opModeIsActive() && !gamepad1.y) ;

        while(opModeIsActive()) {
            drive.turn(Math.toRadians(270));
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("heading Deg.", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
            //while(opModeIsActive() && !gamepad1.y) ;

            drive.turn(Math.toRadians(-270));
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("heading Deg.", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
            //while(opModeIsActive() && !gamepad1.y) ;
        }
    }
}
