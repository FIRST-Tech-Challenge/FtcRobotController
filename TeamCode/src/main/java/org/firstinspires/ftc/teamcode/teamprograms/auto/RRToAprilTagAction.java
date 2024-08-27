package org.firstinspires.ftc.teamcode.teamprograms.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FailedInitializationException;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotVision;

import java.util.concurrent.TimeUnit;

@Autonomous(group = "A")
public class RRToAprilTagAction extends LinearOpMode {




    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);

    RobotVision vision;


    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);

        vision = new RobotVision(hardwareMap, telemetry, true, false);

        while (!isStopRequested() && !vision.isCameraInitialized()) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
        }

        try {
            vision.setManualExposure(6, 200);
        } catch (InterruptedException | FailedInitializationException e) {
            telemetry.addLine("--------------------------------");
            telemetry.addLine(e.getMessage());
            telemetry.addLine("--------------------------------");
        }




        Action driveSequence1 = drive.actionBuilder(startPose)
                .lineToXConstantHeading(12)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(270), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(36, -24), Math.toRadians(0), new TranslationalVelConstraint(30))
                .lineToX(48)
                .build();


        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();


        // run autonomous code

        Actions.runBlocking(new SequentialAction(
                driveSequence1,
                vision.runAprilTagAction(8, 12, RobotVision.Direction.LEFT),
                drive.rebasePose(new Pose2d(72, 0, Math.toRadians(0)))
        ));

        telemetry.addLine("routine complete");
        drive.updatePoseEstimate();
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.addData("Θ", drive.pose.heading.toDouble());
        telemetry.update();

        sleep(10_000);




    }



}
