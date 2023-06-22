package org.firstinspires.ftc.teamcode.commandBased;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
import org.firstinspires.ftc.teamcode.commandBased.commands.intake.SetIntakePower;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.MoveRotatorToPosition;
import org.firstinspires.ftc.teamcode.commandBased.commands.rotator.SetRotatorRange;

@TeleOp(name="Cmd Op Mode", group="Linear Opmode")
public class TestOpMode extends BaseOpMode{

    protected FieldCentric fieldCentric;
    protected RobotCentric robotCentric;
    protected PointCentric pointCentric;

    protected SetDriveSpeeds slowMode;
    protected SetDriveSpeeds fastMode;

    protected ResetGyro resetGyro;

    protected MoveElevatorToPosition eleLow;
    protected MoveElevatorToPosition eleIdle;
    protected MoveElevatorToPosition eleMid;
    protected MoveElevatorToPosition eleHigh;

    protected MoveArmToAngle armBack;
    protected MoveArmToAngle armIdle;
    protected MoveArmToAngle armFront;

    protected UpdateArmPID updateArmPID;

    protected MoveRotatorToPosition rotatorBack;
    protected MoveRotatorToPosition rotatorFront;

    protected SetRotatorRange rotatorRange;

    protected SetIntakePower intakeIntake;
    protected SetIntakePower intakeIdle;
    protected SetIntakePower intakeOuttake;

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

        fieldCentric.schedule();
        rotatorRange.schedule();
        rotatorFront.schedule();
    }

    @Override
    public void run() {
        super.run();

        //drivetrain mode controls
        driver.left_trigger.and(driver.a).onRise(() -> {
            pointCentric.cancel();
            fieldCentric.cancel();
            robotCentric.schedule();
        });
        driver.left_trigger.and(driver.b).onRise(() -> {
            pointCentric.cancel();
            robotCentric.cancel();
            fieldCentric.schedule();
        });
        driver.left_trigger.and(driver.x).onRise(() -> {
            fieldCentric.cancel();
            robotCentric.cancel();
            pointCentric.schedule();
        });

        driver.left_trigger.and(driver.y).onRise(resetGyro::schedule);

        //elevator controls
        driver.dpad_down.onRise(eleLow::schedule);
        driver.dpad_left.onRise(eleIdle::schedule);
        driver.dpad_right.onRise(eleMid::schedule);
        driver.dpad_up.onRise(eleHigh::schedule);

        //arm controls
        driver.b.onRise(armFront::schedule);
        driver.x.onRise(armBack::schedule);
        driver.a.onRise(armIdle::schedule);

        //rotator controls
        driver.back.onRise(() -> rotatorBack.schedule(true));
        driver.back.onRise(rotatorBack::schedule);
        driver.start.onRise(rotatorFront::schedule);

        //intake controls
        driver.left_bumper.onRise(intakeOuttake::schedule)
                .onFall(intakeIdle::schedule);
        driver.right_bumper.onRise(intakeIntake::schedule)
                .onFall(intakeIdle::schedule);

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
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        robotCentric = new RobotCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
                driver.right_stick_x::get
        );
        pointCentric = new PointCentric(
                drivetrainSS,
                driver.left_stick_x::get,
                () -> -driver.left_stick_y.get(),
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
        eleIdle = new MoveElevatorToPosition(elevatorSS, Constants.ELE_MID_LOW);
        eleMid = new MoveElevatorToPosition(elevatorSS, Constants.ELE_MID_HIGH);
        eleHigh = new MoveElevatorToPosition(elevatorSS, Constants.ELE_HIGH);
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
