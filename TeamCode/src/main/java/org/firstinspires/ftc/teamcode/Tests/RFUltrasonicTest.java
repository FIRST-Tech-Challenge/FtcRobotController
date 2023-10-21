package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFUltrasonic;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Components.Ultrasonics;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/**
 * William
 * Program to tune the constants of an RFUltrasonic(x,y,a offset), must have inputable expected obstacle in the form of a linear equation
 */
@Autonomous
public class RFUltrasonicTest extends LinearOpMode {
    public void runOpMode() {
        BasicRobot robot = new BasicRobot(this, false);
        LED ultras = op.hardwareMap.get(LED.class, "ultras");
        RFUltrasonic ultra = new RFUltrasonic("ultra");

        double lastEnabled = 0;
        waitForStart();
        ultras.enable(ultras.isLightOn());
        sleep(50);
        while(!isStopRequested()&&opModeIsActive()) {
            if (op.getRuntime() - lastEnabled >= 0.15) {
                ultras.enable(ultras.isLightOn());
            }

            if (op.getRuntime() - lastEnabled >= 0.2) {

                if (!ultras.isLightOn()) {
                    LOGGER.log(RFLogger.Severity.INFO, "enabled");
                    LOGGER.log(RFLogger.Severity.INFO, ultra.getDist() + "");
                }
                else {
                    LOGGER.log(RFLogger.Severity.INFO, "disabled");
                    op.telemetry.addData("dist: ", ultra.getDist());
                    op.telemetry.update();
                }
                ultras.enable(ultras.isLightOn());

//                op.telemetry.addData("voltage: ", ultra.getVoltage());
//                LOGGER.log(RFLogger.Severity.INFO, currentPose.getX() + "");
//                LOGGER.log(RFLogger.Severity.INFO, currentPose.getY() + "");

                lastEnabled = op.getRuntime();
            }

            robot.update();
        }

//        robot.roadrun.followTrajectorySequence(backdrop);
//        if (realUltras.checkAlliance()) {
//            robot.roadrun.followTrajectorySequence(deposit);
//        }
//        else {
//            robot.roadrun.followTrajectorySequence(wait);
//        }
    }

//    TrajectorySequence backdrop = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -63.25, Math.toRadians(270)))
//            .setReversed(true)
//            .lineToLinearHeading(new Pose2d(-46, -37, toRadians(90)))
//            .setReversed(false)
//            .lineToLinearHeading(new Pose2d(10, -35, toRadians(0)))
//            .addTemporalMarker(robot::done)
//            .build();
//
//    TrajectorySequence wait = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -63.25, Math.toRadians(0)))
//            .lineToConstantHeading(new Vector2d(20, -60))
//            .addTemporalMarker(robot::done)
//            .build();
//
//    TrajectorySequence deposit = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -63.25, Math.toRadians(0)))
//            .lineToLinearHeading(new Pose2d(45, -35, toRadians(0)))
//            .addTemporalMarker(robot::done)
//            .build();
}
