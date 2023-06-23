package org.firstinspires.ftc.teamcode.commandBased.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.classes.CommandSchedulerEx;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.GrabCone;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.LiftMoveRotateArm;
import org.firstinspires.ftc.teamcode.commandBased.commands._groups.ScoreCone;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.MoveArmToAngle;
import org.firstinspires.ftc.teamcode.commandBased.commands.arm.UpdateArmPID;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.FieldCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.PointCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.ResetGyro;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.RobotCentric;
import org.firstinspires.ftc.teamcode.commandBased.commands.drive.SetDriveSpeeds;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.MoveElevatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.elevator.UpdateElevatorPID;
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.SetRotatorRange;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.BaseOpMode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;

@TeleOp
public class Test extends BaseOpMode {

    //drive commands
    protected FieldCentric fieldCentric;
    protected RobotCentric robotCentric;
    protected PointCentric pointCentric;

    protected SetDriveSpeeds slowMode;
    protected SetDriveSpeeds fastMode;

    protected ResetGyro resetGyro;

    //elevator commands
    protected MoveElevatorToPosition eleLow;
    protected MoveElevatorToPosition eleIdle;
    protected MoveElevatorToPosition eleMid;
    protected MoveElevatorToPosition eleHigh;

    protected UpdateElevatorPID updateElevatorPID;

    //arm commands
    protected MoveArmToAngle armBack;
    protected MoveArmToAngle armIdle;
    protected MoveArmToAngle armFront;

    protected UpdateArmPID updateArmPID;

    //rotator commands
    protected MoveRotatorToPosition rotatorBack;
    protected MoveRotatorToPosition rotatorFront;

    protected SetRotatorRange rotatorRange;

    //intake commands
    protected SetIntakePower intakeIntake;
    protected SetIntakePower intakeIdle;
    protected SetIntakePower intakeOuttake;

    //macro commands
    protected ScoreCone scoreCone;
    protected GrabCone grabCone;

    protected LiftMoveRotateArm armFrontMid;
    protected LiftMoveRotateArm armBackMid;
    protected LiftMoveRotateArm armFrontHigh;
    protected LiftMoveRotateArm armBackHigh;

    @Override
    public void initialize() {
        super.initialize();
        initializeAllCommands();

        //speed commands
//        gp1(LEFT_BUMPER, 0).whenActive(slowMode);
//        gp1(RIGHT_BUMPER, 0).whenActive(fastMode);

        //elevator controls
        gp1(DPAD_DOWN, 1).whenActive(eleLow);
        gp1(DPAD_LEFT, 1).whenActive(eleIdle);
        gp1(DPAD_RIGHT, 1).whenActive(eleMid);
        gp1(DPAD_UP, 1).whenActive(eleHigh);

        //arm controls
        gp1(B, 1).whenActive(armFront);
        gp1(A, 1).whenActive(armIdle);
        gp1(X, 1).whenActive(armBack);

        //pid controls
        gp1(LEFT_STICK_BUTTON, 1).whenActive(updateElevatorPID);
        gp1(RIGHT_STICK_BUTTON, 1).whenActive(updateArmPID);

        //rotator controls
        gp1(BACK, 1).whenActive(rotatorBack);
        gp1(START, 1).whenActive(rotatorFront);

        //intake controls
        gp1(LEFT_BUMPER, 1).whenActive(intakeOuttake).whenInactive(intakeIdle);
        gp1(RIGHT_BUMPER, 1).whenActive(intakeIntake).whenInactive(intakeIdle);

        //macro controls
        gp1(Y, 2).whenActive(armBackHigh);
        gp1(A, 2).whenActive(grabCone);

        //drive controls
        gp1(A, 3).toggleWhenActive(robotCentric, fieldCentric);
        gp1(Y, 3).whenActive(resetGyro);

        fieldCentric.schedule();
        rotatorRange.schedule();
        rotatorFront.schedule();
    }

    @Override
    public void run() {
        super.run();
    }


    protected void initializeAllCommands() {
        initializeDriveCommands();
        initializeElevatorCommands();
        initializeArmCommands();
        initializeRotatorCommands();
        initializeIntakeCommands();
        initializeMacroCommands();
    }

    protected void initializeDriveCommands() {
        fieldCentric = new FieldCentric(
                drivetrainSS,
                () -> driver.getLeftX(),
                () -> -driver.getLeftY(),
                () -> driver.getRightX()
        );
        robotCentric = new RobotCentric(
                drivetrainSS,
                () -> driver.getLeftX(),
                () -> -driver.getLeftY(),
                () -> driver.getRightX()
        );
        pointCentric = new PointCentric(
                drivetrainSS,
                () -> driver.getLeftX(),
                () -> -driver.getLeftY(),
                Constants.TARGET,
                Constants.ANGLE_OFFSET
        );

        slowMode = new SetDriveSpeeds(
                drivetrainSS,
                Constants.DRIVE_SLOW_STRAFE,
                Constants.DRIVE_SLOW_FORWARD,
                Constants.DRIVE_SLOW_TURN
        );
        fastMode = new SetDriveSpeeds(
                drivetrainSS,
                Constants.DRIVE_FAST_STRAFE,
                Constants.DRIVE_FAST_FORWARD,
                Constants.DRIVE_FAST_TURN
        );

        resetGyro = new ResetGyro(drivetrainSS);
    }

    protected void initializeElevatorCommands() {
        eleLow = new MoveElevatorToPosition(elevatorSS, Constants.ELE_LOW);
//        CommandSchedulerEx.getInstance().add(eleLow);
        eleIdle = new MoveElevatorToPosition(elevatorSS, Constants.ELE_MID_LOW);
        eleMid = new MoveElevatorToPosition(elevatorSS, Constants.ELE_MID_HIGH);
        eleHigh = new MoveElevatorToPosition(elevatorSS, Constants.ELE_HIGH);

        updateElevatorPID = new UpdateElevatorPID(elevatorSS);
    }

    protected void initializeArmCommands() {
        armBack = new MoveArmToAngle(armSS, Constants.ARM_ANGLE_BACK);
        armIdle = new MoveArmToAngle(armSS, Constants.ARM_ANGLE_IDLE);
        armFront = new MoveArmToAngle(armSS, Constants.ARM_ANGLE_FRONT);

        updateArmPID = new UpdateArmPID(armSS);
    }

    protected void initializeRotatorCommands() {
        rotatorBack = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_BACK);
        rotatorFront = new MoveRotatorToPosition(rotatorSS, Constants.ROTATOR_FRONT);

        rotatorRange = new SetRotatorRange(rotatorSS, Constants.TUNED_RANGE);
    }

    protected void initializeIntakeCommands() {
        intakeIntake = new SetIntakePower(intakeSS, 1);
        intakeIdle = new SetIntakePower(intakeSS, 0);
        intakeOuttake = new SetIntakePower(intakeSS, -0);
    }

    protected void initializeMacroCommands() {
        scoreCone = new ScoreCone(elevatorSS, armSS, rotatorSS, intakeSS);
        grabCone = new GrabCone(elevatorSS, armSS, rotatorSS, intakeSS);
        armFrontMid = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_FRONT,
                Constants.ELE_MID_HIGH,
                Constants.ROTATOR_FRONT
        );
        armBackMid = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_BACK,
                Constants.ELE_MID_HIGH,
                Constants.ROTATOR_BACK
        );
        armFrontHigh = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_FRONT,
                Constants.ELE_HIGH,
                Constants.ROTATOR_FRONT
        );
        armBackHigh = new LiftMoveRotateArm(
                elevatorSS,
                armSS,
                rotatorSS,
                Constants.ARM_ANGLE_BACK,
                Constants.ELE_HIGH,
                Constants.ROTATOR_BACK
        );
    }

}
