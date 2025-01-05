package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;

/**
 * FE!N FE!N FE!N
 */
@Config
@Autonomous(name = "Near Side Park", group = "Autonomous")
public class TestAuton extends LinearOpMode {
    // Create drivetrain object
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    Robot robot;
    @Override
    public void runOpMode() {
        // Set dashboard
        //robot = new Robot(hardwareMap, new Battery(hardwareMap));
        drivetrain = robot.drivetrain;
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        ElapsedTime looptime = new ElapsedTime();
        SimpleMatrix desiredPose = Utils.makePoseVector(0,24,0);
        telemetry.addData("x", 0);
        telemetry.addData("y", 0);
        telemetry.addData("theta", 0);
        telemetry.addData("desiredX", 0);
        telemetry.addData("desiredY", 0);
        telemetry.addData("desiredTheta", 0);
        telemetry.update();
        waitForStart();
        looptime.reset();
        Actions.runBlocking(
            new ParallelAction(
                new SequentialAction(
                drivetrain.goToPose(desiredPose),
                drivetrain.stopMotors()
                ),
                drivetrain.updateTelemetry()
            )
        );
    }
}