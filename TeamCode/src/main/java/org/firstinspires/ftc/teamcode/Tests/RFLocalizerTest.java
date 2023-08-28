package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.LocalizerFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "RFOdometryLocalizerTest")
public class RFLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        Tracker tracker = LocalizerFactory.getTracker(Tracker.TrackType.ODOMETRY);
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        RFMecanumDrive drive2 = new RFMecanumDrive();
        waitForStart();
        while (opModeIsActive()) {
            assert tracker != null;
            tracker.update();
            robot.update();
            drive.update();
            drive2.setJoystickPower(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            packet.put("rrPose", drive.getPoseEstimate());
            packet.put("rrPOVVelocity", drive.getPoseVelocity());
        }
    }
}
