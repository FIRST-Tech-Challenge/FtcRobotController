package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.LocalizerFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "LeftIMULocalizerTest")
public class LeftLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap, Tracker.TrackType.ROADRUN_IMU_LEFT);
        waitForStart();
        while (opModeIsActive()) {
            robot.update();
            drive.update();
            drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
            packet.put("rrPose", drive.getPoseEstimate());
            packet.put("rrPOVVelocity", drive.getPoseVelocity());
        }
    }
}
