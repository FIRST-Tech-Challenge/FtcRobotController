package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.LocalizerFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.Tracker;

@Config
@Autonomous(name = "RFOdometryLocalizerTest")
public class RFLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        Tracker tracker = LocalizerFactory.getChassis(Tracker.TrackType.ODOMETRY);
        waitForStart();
        while (opModeIsActive()) {
            assert tracker != null;
            tracker.update();
            robot.update();
        }
    }
}
