package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

/**
 * Warren Zhou
 * 8/16/23
 * Easily find which motor is right or wrong configured
 */
@Autonomous(name = "MotorTest")
public class MotorTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY);
        waitForStart();
        packet.put("CurrentMotor", "LeftFront");
        robot.update();
        drive.setMotorPowers(0.3, 0, 0, 0);
        sleep(2000);
        packet.put("CurrentMotor", "LeftRear");
        robot.update();
        drive.setMotorPowers(0.0, 0.3, 0, 0);
        sleep(2000);
        packet.put("CurrentMotor", "RightBack");
        robot.update();
        drive.setMotorPowers(0.0, 0, 0.3, 0);
        sleep(2000);
        packet.put("CurrentMotor", "RightFront");
        robot.update();
        drive.setMotorPowers(0, 0, 0, 0.3);
        sleep(2000);
        stop();
    }
}
