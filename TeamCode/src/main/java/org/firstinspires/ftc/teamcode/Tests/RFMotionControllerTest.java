package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive.poseMode;
//import static org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive.isPoseSim;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.LocalizerFactory;
import org.firstinspires.ftc.teamcode.roadrunner.drive.Localizers.Tracker;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFWaypoint;

@Config
@Autonomous(name = "RFMotionControllerTest")
public class RFMotionControllerTest extends LinearOpMode {
    public static double targetX1 = 24, targetY1 = -24, targetX2 = 48, targetY2 = 0, targetX3 = 24, targetY3 = 0, CURVINESS = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this, true);
        RFMecanumDrive drive = new RFMecanumDrive();
        Tracker localizer = null;
        if (poseMode > 1) {
            localizer = LocalizerFactory.getTracker(Tracker.TrackType.ODOMETRY);
        }
        Pose2d startPose = new Pose2d(0, 0, toRadians(90));
        drive.setPose(startPose);
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();
        robot.update();
        BasicRobot.time = 0;
        while (opModeIsActive()) {

//        for(int i=0; i<1;i++) {
            if (!drive.isFollowing()) {
                if (poseMode <= 1) {
                    currentPose = new Pose2d(0, 0, toRadians(0));
                    currentVelocity = new Pose2d(0, 0, 0);
                }
                drive.setReversed(false);
                drive.setCurviness(CURVINESS);
//                drive.setTangentOffset(toRadians(-90));
                drive.addWaypoint(new RFWaypoint(new Vector2d(targetX1, targetY1)));
//                drive.setTangentOffset(toRadians(-90));
//                drive.setReversed(true);

                drive.addWaypoint(new RFWaypoint(new Vector2d(targetX2, targetY2)));
//                drive.setTangentOffset(toRadians(-90));
//                drive.addWaypoint(new RFWaypoint(new Vector2d(targetX3, targetY3)));

            }
            if (poseMode > 1) {
                assert localizer != null;
                localizer.update();
            }
//            sleep(10);
            drive.update();
            robot.update();
        }
//            robot.update();
    }
//    }
}
