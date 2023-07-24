package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFWaypoint;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Config
@Autonomous(name = "RFMotionControllerTest")
public class RFMotionControllerTest extends LinearOpMode {
    public static double targetX1 =25 , targetY1=0, targetX2=50, targetY2=0, targetX3=75,targetY3=0;
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        RFMecanumDrive drive = new RFMecanumDrive();
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPose(startPose);
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();
        robot.update();
        BasicRobot.time = 0;
        while (opModeIsActive()) {
//        for(int i=0; i<1;i++) {
            if (!drive.isFollowing()) {
                currentPose = new Pose2d (0,0,0);
                currentVelocity = new Pose2d(0,0,0);
                drive.setReversed(false);
                drive.setCurviness(0.5);
                drive.addWaypoint(new RFWaypoint(new Vector2d(targetX1, targetY1)));
                drive.addWaypoint(new RFWaypoint(new Vector2d(targetX2, targetY2)));
                drive.addWaypoint(new RFWaypoint(new Vector2d(targetX3, targetY3)));

            }
            drive.update();
            robot.update();
        }
//            robot.update();
        }
//    }
}
