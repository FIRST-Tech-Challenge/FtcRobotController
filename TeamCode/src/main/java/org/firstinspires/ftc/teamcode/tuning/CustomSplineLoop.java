package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "C")
public class CustomSplineLoop extends LinearOpMode {

    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0,0,0);


    public void runOpMode() {



        drive = new MecanumDrive(hardwareMap, startPose);


//        Action actionSequenceSpline = drive.actionBuilder(startPose)
//                .splineTo(new Vector2d(24,24), Math.toRadians(0))
//                .setReversed(true)
//                .splineTo(new Vector2d(0,0), Math.toRadians(180))
//                .build();




        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


//            Actions.runBlocking(actionSequenceSpline);

            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineTo(new Vector2d(24,24), Math.toRadians(0))
                            .setReversed(true)
                            .splineTo(new Vector2d(0,0), Math.toRadians(180))
                            .build());



            // update telemetry
            drive.updatePoseEstimate();
            telemetry.addLine("Path End Readings");
            telemetry.addData("X", drive.pose.position.x);
            telemetry.addData("Y", drive.pose.position.y);
            telemetry.addData("Θ", drive.pose.heading.toDouble());
            telemetry.update();
            sleep(1_000);
        }



    }

}
