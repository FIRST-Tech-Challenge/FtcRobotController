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
public class PathTangent extends LinearOpMode {

    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);



    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);


        Action actionSequenceTangent = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .lineToY(24)
                .setTangent(Math.toRadians(0))
                .lineToX(24)
                        .build();




        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();


        // auto code
        Actions.runBlocking(actionSequenceTangent);




        // update telemetry and end
        drive.updatePoseEstimate();
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.addData("Θ", drive.pose.heading.toDouble());
        telemetry.update();
        sleep(10000);


    }
}
