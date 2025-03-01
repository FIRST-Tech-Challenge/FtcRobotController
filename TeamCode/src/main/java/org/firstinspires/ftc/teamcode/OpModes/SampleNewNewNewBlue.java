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
import org.firstinspires.ftc.teamcode.Mechanisms.Sweeper.Sweeper;

@Config
@Autonomous(name = "Sample v4 BLUE", group = "AAAAAAAA")
public class SampleNewNewNewBlue extends LinearOpMode {
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
    Sweeper sweeper;
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
        sweeper = robot.sweeper;
        drivetrain.setInitialPose(-63,-33,0);
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
                        drivetrain.goToPose(Utils.makePoseVector(-57, -31,0)),
                        new ParallelAction(
                                drivetrain.goToPose(Utils.makePoseVector(-57, -17,-43)),
                                lift.moveToHeight(24),
                                new SequentialAction(
                                        new SleepAction(.55),
                                        arm.armExtend()
                                )
                        ),
                        new SleepAction(.3),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                lift.moveToHeight(0),
                                new SequentialAction(
                                        new ParallelAction(
                                                drivetrain.goToPose(Utils.makePoseVector(-50,-14,-20))),
                                        new ParallelAction(
                                                robot.intakeDown(),
                                                drivetrain.goToPose(Utils.makePoseVector(-47,-15,-20))))),
                        new SleepAction(0.45),
                        pivot.setPosition(Intake.intakeState.STOP),
                        extension.servoExtension(Extension.extensionState.RETRACT),
                        new SleepAction(0.5),
                        intake.motorIntake(Intake.intakeState.STOP),
                        arm.armRetract(),
                        new SleepAction(0.3),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        new ParallelAction(
                                drivetrain.goToPose(Utils.makePoseVector(-57, -17,-43)),
                                lift.moveToHeight(24),
                                new SequentialAction(
                                        new SleepAction(.4),
                                        arm.armExtend()
                                )
                        ),
                        new SleepAction(.3),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                lift.moveToHeight(0),
                                new SequentialAction(
                                        new ParallelAction(
                                                drivetrain.goToPose(Utils.makePoseVector(-50,-22,21))),
                                        new ParallelAction(
                                                robot.intakeDown(),
                                                drivetrain.goToPose(Utils.makePoseVector(-47,-20.5,21))))),
                        new SleepAction(0.45),
                        pivot.setPosition(Intake.intakeState.STOP),
                        extension.servoExtension(Extension.extensionState.RETRACT),
                        new SleepAction(0.5),
                        intake.motorIntake(Intake.intakeState.STOP),
                        arm.armRetract(),
                        new SleepAction(0.3),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        new ParallelAction(
                                drivetrain.goToPose(Utils.makePoseVector(-57, -17,-43)),
                                lift.moveToHeight(24),
                                new SequentialAction(
                                        new SleepAction(.4),
                                        arm.armExtend()
                                )
                        ),
                        new SleepAction(.3),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                lift.moveToHeight(0),
                                //drivetrain.goToPose(Utils.makePoseVector(-42,-16.5,44)),
                                drivetrain.goToPose(Utils.makePoseVector(-42,-15.5,35))),
                        robot.intakeDown(),
                        new SleepAction(0.45),
                        pivot.setPosition(Intake.intakeState.STOP),
                        extension.servoExtension(Extension.extensionState.RETRACT),
                        new SleepAction(0.5),
                        intake.motorIntake(Intake.intakeState.STOP),
                        arm.armRetract(),
                        new SleepAction(0.3),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        new ParallelAction(
                                drivetrain.goToPose(Utils.makePoseVector(-57, -17,-43)),
                                lift.moveToHeight(24),
                                new SequentialAction(
                                        new SleepAction(.75),
                                        arm.armExtend()
                                )
                        ),
                        new SleepAction(.6),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(0.3),
                        arm.armNeutral(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                lift.moveToHeight(0),
                                drivetrain.goToPose(Utils.makePoseVector(-13, -35, -90)),
                                arm.armNeutral()),
                        drivetrain.goToPoseImpresice(Utils.makePoseVector(-9, -50, -90)),
                        extension.midPos(),
                        sweeper.sweep(),
                        new SleepAction(.4),
                        sweeper.sweep(),
                        pivot.setPosition(Intake.intakeState.INTAKE),
                        intake.motorIntake(Intake.intakeState.INTAKE),
                        new ParallelAction(

                        new SleepAction(1),
                                robot.stopIfRed(),
                                /*new SequentialAction(
                                drivetrain.goToPose(Utils.makePoseVector(-8, -51, -85)),
                                drivetrain.goToPose(Utils.makePoseVector(-8, -49, -95))
                                ),*/
                                extension.servoExtension(Extension.extensionState.EXTEND)
                        ),
                        extension.servoExtension(Extension.extensionState.RETRACT),
                        pivot.setPosition(Intake.intakeState.STOP),
                        //drivetrain.goToPose(Utils.makePoseVector(-13, -35, -90)),
                        intake.motorIntake(Intake.intakeState.STOP),
                        new SleepAction(0.5),
                        arm.armRetract(),
                        new SleepAction(0.5),
                        claw.servoClaw(Claw.clawState.CLOSE),
                        new ParallelAction(
                                drivetrain.goToPose(Utils.makePoseVector(-56, -17,-43)),
                                lift.moveToHeight(24)),
                                new SequentialAction(
                                        new SleepAction(.5),
                                        arm.armExtend()
                                ),
                        new SleepAction(.3),
                        claw.servoClaw(Claw.clawState.OPEN),
                        new SleepAction(.4),
                        arm.armRetract(),
                        new SleepAction(.4),
                        lift.moveToHeight(0)
                )
        );
    }
}