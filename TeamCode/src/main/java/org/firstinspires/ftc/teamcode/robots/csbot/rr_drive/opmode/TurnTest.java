package org.firstinspires.ftc.teamcode.robots.csbot.rr_drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.csbot.rr_drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "Turn Test CS", group = "test")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.turn(Math.toRadians(ANGLE));
        telemetry.addData("Left Front Motor Position:\t", drive.leftFront.getCurrentPosition());
        telemetry.addData("Left Back Motor Position:\t", drive.leftRear.getCurrentPosition());
        telemetry.addData("Right Front Motor Position:\t", drive.rightFront.getCurrentPosition());
        telemetry.addData("Right Back Motor Position:\t", drive.rightRear.getCurrentPosition());
        telemetry.update();
    }
}
