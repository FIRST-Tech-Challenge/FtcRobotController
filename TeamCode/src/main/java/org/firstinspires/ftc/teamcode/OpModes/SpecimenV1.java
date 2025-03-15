package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;
import org.firstinspires.ftc.teamcode.Mechanisms.Sweeper.Sweeper;

@Config
@Autonomous(name = "Specimenn", group = "AAAAAAAAAA")
public class SpecimenV1 extends LinearOpMode{
        FtcDashboard dashboard;
        Robot robot;
        Drivetrain drivetrain;
        Extension extension;
        Intake intake;
        Arm arm;
        Claw claw;
        Pivot pivot;
        Lift lift;
        Sweeper sweeper;

        @Override
        public void runOpMode() {
            // Set dashboard
            //robot = new Robot(hardwareMap, new Battery(hardwareMap));
            dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
            ElapsedTime looptime = new ElapsedTime();
            robot = new Robot(hardwareMap);
            drivetrain = robot.drivetrain;
            extension = robot.extension;
            intake = robot.intake;
            arm = robot.arm;
            claw = robot.claw;
            pivot = robot.pivot;
            lift = robot.lift;
            sweeper = robot.sweeper;
            drivetrain.setInitialPose(-63,20,180);
            telemetry.addData("X", 0);
            telemetry.addData("Y", 0);
            telemetry.addData("Theta", 0);
            telemetry.addData("X Velocity", 0);
            telemetry.addData("Y Velocity", 0);
            telemetry.addData("Theta Velocity", 0);
            telemetry.update();
            waitForStart();
            looptime.reset();
            Actions.runBlocking(
                        new SequentialAction(
                        arm.specimenAuton(),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        drivetrain.goToPose(Utils.makePoseVector(-62,19,180))
                    )
            );
        }
    }
