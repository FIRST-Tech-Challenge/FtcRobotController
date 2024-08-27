package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "C")
public class CustomDistanceTest extends LinearOpMode {

    /*
    This opmode is meant for testing the feedforward K variables to determine
    if they are tuned accurately

    DO NOT have the feedback PID variables set to anything when testing this, otherwise
    they will override and try to forcefully correct the errors made by feedforward
     */

    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0, 0, 0);

    static int DIST = 64;


    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);

        Action driveActionSequence = drive.actionBuilder(drive.pose)
                .lineToX(64)
                .build();

        telemetry.addLine("Press start to test feedforward accuracy.");
        telemetry.addData("DIST:", DIST);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(driveActionSequence);

        drive.updatePoseEstimate();
        telemetry.addData("Distance traveled:", drive.pose.position.x);
        telemetry.update();

        sleep(10000);


    }

}
