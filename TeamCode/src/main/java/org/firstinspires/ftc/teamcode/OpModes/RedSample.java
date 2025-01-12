package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
                        drivetrain.goToPose(Utils.makePoseVector(-57, -36,0)),
                        drivetrain.goToPose(Utils.makePoseVector(-61, -16.5,-45)),

                            intake.motorIntake(Intake.intakeState.INTAKE),
                            pivot.setPosition(Intake.intakeState.INTAKE),
                            extension.servoExtension(Extension.extensionState.EXTEND),
                            drivetrain.goToPose(Utils.makePoseVector(-51,-23.5,0)),
                            arm.armNeutral(),

                        drivetrain.goToPose(Utils.makePoseVector(-49.5,-23.5,0)),
                        pivot.setPosition(Intake.intakeState.STOP),
                        new SleepAction(0.3),
                        // waits for intake to grab
                            claw.servoClaw(Claw.clawState.OPEN),
                            extension.servoExtension(Extension.extensionState.RETRACT),
                        new SleepAction(0.5),
                        arm.armRetract(),
                        intake.motorIntake(Intake.intakeState.STOP),
                        new SleepAction(0.3),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        arm.armExtend(),
                        new SleepAction(0.3),
                        //lift,
                        claw.servoClaw(Claw.clawState.OPEN),
                        pivot.setPosition(Intake.intakeState.INTAKE),
                        drivetrain.goToPose(Utils.makePoseVector(-61, -16.5,-45)),
                        drivetrain.goToPose(Utils.makePoseVector(-51,-13.5,0)),
                        extension.servoExtension(Extension.extensionState.EXTEND),
                        intake.motorIntake(Intake.intakeState.INTAKE),
                        drivetrain.goToPose(Utils.makePoseVector(-49.5,-13.5,0)),
                        new SleepAction(1),
                        pivot.setPosition(Intake.intakeState.STOP),
                        intake.motorIntake(Intake.intakeState.STOP),
                        claw.servoClaw(Claw.clawState.OPEN),
                        arm.armNeutral(),
                        extension.servoExtension(Extension.extensionState.RETRACT),
                        new SleepAction(0.5),
                        arm.armRetract(),
                        new SleepAction(1),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        new SleepAction(0.3),
                        arm.armExtend(),
                        //lift,
                        claw.servoClaw(Claw.clawState.OPEN),
                        arm.armExtend()

                )
        );
    }
}
