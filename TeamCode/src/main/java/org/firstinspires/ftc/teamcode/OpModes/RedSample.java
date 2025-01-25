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

import org.firstinspires.ftc.teamcode.Mechanisms.Arm.Arm;
import org.firstinspires.ftc.teamcode.Mechanisms.Claw.Claw;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Utils.Utils;
import org.firstinspires.ftc.teamcode.Mechanisms.Extension.Extension;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Lift.Lift;
import org.firstinspires.ftc.teamcode.Mechanisms.Pivot.Pivot;
import org.firstinspires.ftc.teamcode.Mechanisms.Robot.Robot;

@Config
@Autonomous(name = "Red Sample", group = "Autonomous")
public class RedSample extends LinearOpMode {

    // Use FTCDashboard
    FtcDashboard dashboard;
    Robot robot;
    Drivetrain drivetrain;
    Extension extension;
    Intake intake;
    Arm arm;
    Claw claw;
    Pivot pivot;
    Lift lift;
    @Override
    public void runOpMode() {
        // Set dashboard
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
        drivetrain.setInitialPose(-63,-36,0);
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
                        robot.intakeUp(),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        arm.armNeutral(),
                        drivetrain.goToPose(Utils.makePoseVector(-57, -36,0)),
                        drivetrain.goToPose(Utils.makePoseVector(-57, -17,-46.5)),
                        lift.moveToHeight(28),
                        new SleepAction(1),
                        arm.armExtend(),
                        new SleepAction(1),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        new SleepAction(0.3),
                        lift.moveToHeight(0),
                        new ParallelAction(
                            robot.intakeDown(),
                            drivetrain.goToPose(Utils.makePoseVector(-51,-23.5,0))
                        ),
                        drivetrain.goToPose(Utils.makePoseVector(-49,-23.5,0)),
                        new SleepAction(0.85),
                        robot.intakeUp(),
                        new SleepAction(0.5),
                        arm.armRetract(),
                        new SleepAction(0.3),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        drivetrain.goToPose(Utils.makePoseVector(-57, -19,-43)),
                        lift.moveToHeight(28),
                        new SleepAction(1),
                        arm.armExtend(),
                        new SleepAction(1),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        new SleepAction(0.3),
                        lift.moveToHeight(0),
                        drivetrain.goToPose(Utils.makePoseVector(-51,-13.5,0)),
                        robot.intakeDown(),
                        drivetrain.goToPose(Utils.makePoseVector(-49.5,-13.5,0)),
                        new SleepAction(1),
                        robot.intakeUp(),
                        new SleepAction(0.5),
                        arm.armRetract(),
                        new SleepAction(0.3),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        drivetrain.goToPose(Utils.makePoseVector(-57, -17,-46.5)),
                        lift.moveToHeight(28),
                        new SleepAction(1),
                        arm.armExtend(),
                        new SleepAction(1),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        new SleepAction(0.3),
                        lift.moveToHeight(0)
                )
        );
    }
}
