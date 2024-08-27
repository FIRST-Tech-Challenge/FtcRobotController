package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(group = "R")
public class Path1 extends LinearOpMode {

    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0, 0, 0);
    Pose2d rebasePose = new Pose2d(10, 10, 0);
    Servo servo;

    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);

        servo = hardwareMap.get(Servo.class, "servo");

        Action actionSequence1 = drive.actionBuilder(startPose)
                .lineToX(24)
                .splineToSplineHeading(new Pose2d(36, 24, Math.toRadians(0)), Math.toRadians(90))
                .build();


        Action actionSequence2 = drive.actionBuilder(rebasePose)
                .lineToX(22)
                .splineTo(new Vector2d(46, -14), Math.toRadians(0))
                .build();




        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        // auto code
        servo.setPosition(0);

        Actions.runBlocking(actionSequence1);

        servo.setPosition(0.5);
        drive.pose = rebasePose;
        sleep(1000);

        Actions.runBlocking(actionSequence2);







        // update telemetry and end
        drive.updatePoseEstimate();
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.addData("Θ", drive.pose.heading.toDouble());
        telemetry.update();
        sleep(10000);


    }


}
