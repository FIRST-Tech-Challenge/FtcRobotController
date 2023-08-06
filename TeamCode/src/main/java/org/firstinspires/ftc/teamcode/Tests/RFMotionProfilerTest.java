package org.firstinspires.ftc.teamcode.Tests;


import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPos;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
@Config
@Autonomous(name = "RFMotionProfilerTest")
public class RFMotionProfilerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BasicRobot robot = new BasicRobot(this,true);
        RFMotor motor = new RFMotor("simMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false, 10000, 0);
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        motor.setPose(startPose);
        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();
        robot.update();
        BasicRobot.time = 0;
        while (opModeIsActive()) {
            currentPos = 0;
            currentVelocity = 0;

        }
        motor.update();
        robot.update();

    }
}
