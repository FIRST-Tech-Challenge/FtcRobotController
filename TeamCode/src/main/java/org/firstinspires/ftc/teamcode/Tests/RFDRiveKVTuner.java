package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
//import static org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive.isPoseSim;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.LocalizerFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.OdometryTracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFWaypoint;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Config
@Autonomous(name = "RFDriveKVTuner")
public class RFDRiveKVTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        RFMecanumDrive drive = new RFMecanumDrive();
        Tracker localizer = new OdometryTracker();
        waitForStart();
        double lastTime = BasicRobot.time;
        while(opModeIsActive()){
            lastTime = BasicRobot.time;
            while(BasicRobot.time<lastTime+2){
                drive.setDriveVelocity(new Pose2d(30,0,0));
                robot.update();
                localizer.update();
                drive.update();
            }
            drive.setReversed(true);
            drive.addWaypoint(new RFWaypoint(new Vector2d(0,0)));
            while(drive.isFollowing()){
                robot.update();
                localizer.update();
                drive.update();
            }
        }
//            robot.update();
    }
//    }
}
