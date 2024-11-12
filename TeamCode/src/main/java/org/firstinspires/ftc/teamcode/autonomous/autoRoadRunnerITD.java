package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.teleop.MecanumDrive;
@Config
@Autonomous
public class autoRoadRunnerITD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Servo specsServo = hardwareMap.get(Servo.class, "specsServo");
        //DcMotor rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        Pose2d beginPose = new Pose2d(0, -61.5, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder trajectory = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(0,-31))
                .waitSeconds(1)
                .strafeTo(new Vector2d(0,-34))
                .strafeTo(new Vector2d(35, -35))
                .strafeTo(new Vector2d(35, -10))
                .splineTo(new Vector2d(47, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(47, -51))
                .strafeTo(new Vector2d(47, -58))
                .strafeTo(new Vector2d(47, -45))
                .strafeTo(new Vector2d(47, -60.5))
                .waitSeconds(2)
                .strafeTo(new Vector2d(47, -45))
                .splineTo(new Vector2d(2, -52), Math.toRadians(270))
                .strafeTo(new Vector2d(2, -31))
                .waitSeconds(1)
                .splineTo(new Vector2d(47, -47), Math.toRadians(90))
                .strafeTo(new Vector2d(47,-61));
                Action trajectoryAction = trajectory.build();


        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction
                )
        );
    }
}