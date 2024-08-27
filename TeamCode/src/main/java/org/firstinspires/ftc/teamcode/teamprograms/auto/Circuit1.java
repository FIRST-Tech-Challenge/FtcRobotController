package org.firstinspires.ftc.teamcode.teamprograms.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "R")
public class Circuit1 extends LinearOpMode {

    MecanumDrive drive;

    Pose2d startPose = new Pose2d(0, 0, 0);


    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);

        Action circuitStart = drive.actionBuilder(startPose)
                .lineToX(12)
                .splineTo(new Vector2d(24, -12), Math.toRadians(270))
                .lineToY(-36)
                .splineTo(new Vector2d(36, -48), Math.toRadians(0))
                .lineToX(60)
                .splineTo(new Vector2d(72, -36), Math.toRadians(90))
                .splineTo(new Vector2d(60, -24), Math.toRadians(180))
                .lineToX(12)
                .splineTo(new Vector2d(0, -12), Math.toRadians(90))
                .splineTo(new Vector2d(12, 0), Math.toRadians(0))
                        .build();

        

        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();


        // run RR path
        Actions.runBlocking(circuitStart);
        showCoordinates();



        while (opModeIsActive()) {

            // run circuit loop
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(12, 0, 0))
                            .splineTo(new Vector2d(24, -12), Math.toRadians(270))
                            .lineToY(-36)
                            .splineTo(new Vector2d(36, -48), Math.toRadians(0))
                            .lineToX(60)
                            .splineTo(new Vector2d(72, -36), Math.toRadians(90))
                            .splineTo(new Vector2d(60, -24), Math.toRadians(180))
                            .lineToX(12)
                            .splineTo(new Vector2d(0, -12), Math.toRadians(90))
                            .splineTo(new Vector2d(12, 0), Math.toRadians(0))
                            .build());

            showCoordinates();
        }


    } // end opMode

    private void showCoordinates() {
        // update telemetry
        drive.updatePoseEstimate();
        telemetry.addData("X", drive.pose.position.x);
        telemetry.addData("Y", drive.pose.position.y);
        telemetry.addData("Θ", drive.pose.heading.toDouble());
        telemetry.update();
        sleep(5_000);
    }

}
