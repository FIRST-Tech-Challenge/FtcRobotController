package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(group = "C")
public class CustomTurnTest extends LinearOpMode {

    /*
    This opmode is meant for the tuning of the trackWidthTicks parameter
    within the MecanumDrive class;

    DO NOT have the feedback PID variables set to anything when testing this, otherwise
    they will override and try to forcefully correct the errors made by feedforward
     */

    MecanumDrive drive;
    Pose2d startPose = new Pose2d(0, 0, 0);


    @Override
    public void runOpMode() {

        drive = new MecanumDrive(hardwareMap, startPose);

        Action turnActionSequence = drive.actionBuilder(drive.pose)
                .turn(Math.toRadians(180))
                .turn(Math.toRadians(180))
                .turn(Math.toRadians(180))
                .turn(Math.toRadians(180))
                .turn(Math.toRadians(180))
                .build();

        telemetry.addLine("Press start to test trackWidthTicks parameter accuracy.");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(turnActionSequence);

        drive.updatePoseEstimate();
        telemetry.addData("HEADING:", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        sleep(10000);


    }

}
