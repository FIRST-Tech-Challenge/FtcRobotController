package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(group = "R")
public class PathTemp extends LinearOpMode {


    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);

    Servo servo;

    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);

        servo = hardwareMap.get(Servo.class, "servo");


        Action actionSequenceTemp = drive.actionBuilder(startPose)
                .lineToX(12)
                .lineToX(24)
                .afterTime(0.5, () -> {
                    servo.setPosition(0.5);
                })
                .splineTo(new Vector2d(48, 24), Math.toRadians(0))
                        .build();


        servo.setPosition(0);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();


        // auto code
        Actions.runBlocking(actionSequenceTemp);



        // update telemetry and end
        drive.updatePoseEstimate();
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.addData("Θ", drive.pose.heading.toDouble());
        telemetry.update();
        sleep(10000);



    }

}
