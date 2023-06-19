package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandGroup;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.SetDriveSpeeds;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.PointCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.SetRotatorRange;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;
import org.firstinspires.ftc.teamcode.rr.util.DashboardUtil;


import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;
import ftc.rogue.blacksmith.listeners.ReforgedGamepad;

@Config
@TeleOp(name="Command Based", group="Linear Opmode")
public class Robot extends BlackOp {

    @Override
    public void go() {

        //cancel all previous commands
        CommandScheduler.getInstance().reset();

        //create subsystem objects
        DrivetrainSubsystem drivetrainSS = new DrivetrainSubsystem(hardwareMap);
        ElevatorSubsystem elevatorSS = new ElevatorSubsystem(hardwareMap);
        ArmSubsystem armSS = new ArmSubsystem(hardwareMap);
        RotatorSubsystem rotatorSS = new RotatorSubsystem(hardwareMap);
        IntakeSubsystem intakeSS = new IntakeSubsystem(hardwareMap);

        //create gamepads
        ReforgedGamepad driver = new ReforgedGamepad(gamepad1);
        ReforgedGamepad operator = new ReforgedGamepad(gamepad2);

        //create drivetrain mode commands
        FieldCentric fieldCentric = new FieldCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        RobotCentric robotCentric = new RobotCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        PointCentric pointCentric = new PointCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                Constants.TARGET,
                Constants.ANGLE_OFFSET
        );

        //create drivetrain speed commands
        SetDriveSpeeds slowMode = new SetDriveSpeeds(
                drivetrainSS,
                Constants.DRIVE_SLOW_STRAFE,
                Constants.DRIVE_SLOW_FORWARD,
                Constants.DRIVE_SLOW_TURN
        );
        SetDriveSpeeds fastMode = new SetDriveSpeeds(
                drivetrainSS,
                Constants.DRIVE_FAST_STRAFE,
                Constants.DRIVE_FAST_FORWARD,
                Constants.DRIVE_FAST_TURN
        );

        //create elevator commands
        MoveElevatorToPosition eleLow = new MoveElevatorToPosition(elevatorSS, Constants.ELE_LOW);
        MoveElevatorToPosition eleMidLow = new MoveElevatorToPosition(elevatorSS, Constants.ELE_MID_LOW);
        MoveElevatorToPosition eleMidHigh = new MoveElevatorToPosition(elevatorSS, Constants.ELE_MID_HIGH);
        MoveElevatorToPosition eleHigh = new MoveElevatorToPosition(elevatorSS, Constants.ELE_HIGH);

        //create arm commands
        MoveArmToAngle armBackward = new MoveArmToAngle(
                armSS,
                Constants.ARM_ANGLE_BACK,
                Constants.ARM_MAX_VELO,
                Constants.ARM_MAX_ACCEL
        );
        MoveArmToAngle armIdle = new MoveArmToAngle(
                armSS,
                Constants.ARM_ANGLE_IDLE,
                Constants.ARM_IDLE_VELO,
                Constants.ARM_IDLE_ACCEL
        );
        MoveArmToAngle armForward = new MoveArmToAngle(
                armSS,
                Constants.ARM_ANGLE_FRONT,
                Constants.ARM_MAX_VELO,
                Constants.ARM_MAX_ACCEL
        );

        //create rotator commands
        MoveRotatorToPosition rotatorBack = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_BACK);
        MoveRotatorToPosition rotatorFront = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_FORWARD);

        SetRotatorRange rotatorRange = new SetRotatorRange(rotatorSS, Constants.TUNED_RANGE);

        //create intake commands
        SetIntakePower intakeIntake = new SetIntakePower(intakeSS, 1);
        SetIntakePower intakeIdle = new SetIntakePower(intakeSS, 0);
        SetIntakePower intakeOuttake = new SetIntakePower(intakeSS, -1);

        //create group commands
        LiftMoveRotateArm armBackHigh = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_BACK,
                Constants.ARM_MAX_VELO,
                Constants.ARM_MAX_ACCEL,
                Constants.ELE_HIGH,
                Constants.ROTATOR_BACK
        );

        
        

        //start robot in field-centric mode
        robotCentric.schedule();
        rotatorRange.schedule();
        rotatorFront.schedule();

        waitForStart();



        Scheduler.launchOnStart(this, () -> {

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            //activate scheduler
            CommandScheduler.getInstance().run();

            //drivetrain speed controls
            driver.left_bumper.onRise(slowMode::schedule)
                              .onFall(fastMode::schedule);

            //drivetrain mode controls
            driver.a.onRise(() -> {
                pointCentric.cancel();
                fieldCentric.cancel();
                robotCentric.schedule();
            });
            driver.b.onRise(() -> {
                pointCentric.cancel();
                robotCentric.cancel();
                fieldCentric.schedule();
            });
            driver.x.onRise(() -> {
                fieldCentric.cancel();
                robotCentric.cancel();
                pointCentric.schedule();
            });

            driver.y.onRise(drivetrainSS::resetGyro);

            //elevator controls
            driver.dpad_down.onRise(eleLow::schedule);
            driver.dpad_left.onRise(eleMidLow::schedule);
            driver.dpad_right.onRise(eleMidHigh::schedule);
            driver.dpad_up.onRise(eleHigh::schedule);

            //arm controls
            driver.b.onRise(armForward::schedule);
            driver.x.onRise(armBackward::schedule);
            driver.a.onRise(armIdle::schedule);

            //rotator controls
            driver.back.onRise(rotatorBack::schedule);
            driver.start.onRise(rotatorFront::schedule);

            //intake controls
            driver.left_bumper.onRise(intakeOuttake::schedule)
                              .onFall(intakeIdle::schedule);
            driver.right_bumper.onRise(intakeIntake::schedule)
                               .onFall(intakeIdle::schedule);

            driver.right_stick_button.onRise(armBackHigh::schedule);

            // Draw the target on the field
            fieldOverlay.setStroke("#dd2c00");
            fieldOverlay.strokeCircle(Constants.TARGET.getX(), Constants.TARGET.getY(), 3);

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, drivetrainSS.getPose());

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (Constants.DEBUG_DRIVE) {

            }

            if (Constants.DEBUG_ELE) {
                mTelemetry().addData("ele pos", elevatorSS.getElePos());
                mTelemetry().addData("ele profile target", elevatorSS.getEleProfileTarget());
                mTelemetry().addData("ele final target", elevatorSS.getEleTarget());
                mTelemetry().addData("ele power", elevatorSS.getElePower());
            }

            if (Constants.DEBUG_ARM) {
                mTelemetry().addData("arm final encoder target", armSS.getArmTargetEnc());
                mTelemetry().addData("arm final angle target", armSS.getArmTargetAngle());
                mTelemetry().addData("arm profile target", armSS.getArmProfileTarget());
                mTelemetry().addData("arm target", armSS.getArmTargetEnc());
                mTelemetry().addData("arm pos", armSS.getArmPos());
                mTelemetry().addData("arm power", armSS.getArmPower());
                mTelemetry().addData("arm angle", armSS.getArmAngle());
                mTelemetry().addData("arm velocity", armSS.getArmVelocity());
                mTelemetry().addData("arm acceleration", armSS.getArmAcceleration());
                mTelemetry().addData("arm KF", armSS.getCoeffs()[6]);
            }

            if (Constants.DEBUG_ROTATOR) {
                mTelemetry().addData("rotator pos", rotatorSS.getPosition());
                mTelemetry().addData("rotator usFrame", rotatorSS.getPWMRange()[0]);
                mTelemetry().addData("rotator usPulseLower", rotatorSS.getPWMRange()[1]);
                mTelemetry().addData("rotator usPulseUpper", rotatorSS.getPWMRange()[2]);
                //mTelemetry().addData("rotator current", rotatorSS.getCurrent());
            }

            if (Constants.DEBUG_INTAKE) {
                mTelemetry().addData("intake power", intakeSS.getPower());
                mTelemetry().addData("intake current", intakeSS.getServoBusCurrent());
            }

            mTelemetry().addData("group scheduled", armBackHigh.isScheduled());

            mTelemetry().addData("ele scheduled", armBackHigh.moveEle.isScheduled());
            mTelemetry().addData("ele triggered", armBackHigh.moveEle.isTriggered());
            mTelemetry().addData("ele finished", armBackHigh.moveEle.isFinished());

            mTelemetry().addData("arm scheduled", armBackHigh.moveArm.isScheduled());
            mTelemetry().addData("arm triggered", armBackHigh.moveArm.isTriggered());
            mTelemetry().addData("arm finished", armBackHigh.moveArm.isFinished());

            mTelemetry().addData("rotator scheduled", armBackHigh.moveRot.isScheduled());
            mTelemetry().addData("rotator triggered", armBackHigh.moveRot.isTriggered());
            mTelemetry().addData("rotator finished", armBackHigh.moveRot.isFinished());

            mTelemetry().addData("cmdIndex", armBackHigh.getCmdIndex());
            mTelemetry().addData("test", armBackHigh.getTest());
            mTelemetry().addData("name", armBackHigh.getCommandName());

            mTelemetry().update();
        });
    }
}
