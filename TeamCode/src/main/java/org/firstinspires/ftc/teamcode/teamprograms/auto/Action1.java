package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Action1 extends LinearOpMode {

    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);
    Servo servo;

    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);
        servo = hardwareMap.get(Servo.class, "servo");

        Action actionSequence1 = drive.actionBuilder(startPose)
                .build();


    }

}
