package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.ExtendIntake;
import org.firstinspires.ftc.teamcode.commands.ExtendIntakeVariable;
import org.firstinspires.ftc.teamcode.commands.ManualElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.commands.ScoreAtBucket;
import org.firstinspires.ftc.teamcode.commands.SetKickerPosition;
import org.firstinspires.ftc.teamcode.commands.SetRollerState;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRoller;
@TeleOp
public class SampleOpMode extends CommandOpMode {

    private GamepadEx driver;
    private GamepadEx operator;
    private Arm arm;
    private Intake intake;
    private IntakeRoller intakeRoller;
    private Elevator elevator;
    private Drivetrain drivetrain;
    private Claw claw;

    @Override
    public void initialize() {
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        arm = new Arm(this.hardwareMap);
        intake = new Intake(this.hardwareMap);
        elevator = new Elevator(this.hardwareMap, telemetry);
        drivetrain = new Drivetrain(this.hardwareMap, new Pose2d(-58.923881554, -55.0502525317, Math.toRadians(180)), telemetry);
        intakeRoller = new IntakeRoller(hardwareMap);
        claw = new Claw(hardwareMap);

        GamepadButton armButton = new GamepadButton(
                operator, GamepadKeys.Button.A
        );
        GamepadButton clawButton = new GamepadButton(
                operator, GamepadKeys.Button.X
        );

        GamepadButton intakeButton = new GamepadButton(
                driver, GamepadKeys.Button.RIGHT_BUMPER
        );

        GamepadButton outtakeButton = new GamepadButton(
                driver, GamepadKeys.Button.LEFT_BUMPER
        );

        GamepadButton elevatorUpButton = new GamepadButton(
                operator, GamepadKeys.Button.LEFT_BUMPER
        );

        GamepadButton elevatorDownButton = new GamepadButton(
                operator, GamepadKeys.Button.RIGHT_BUMPER
        );

        GamepadButton kickerButton = new GamepadButton(
                driver, GamepadKeys.Button.A
        );
        // You can compose triggers to bind multiple buttons to one action
        // if the trigger is held, move the intake based on the trigger
        Trigger extendIntake = new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);

        // If and only if the intake button is pressed (no trigger) deploy the intake to just in front of the robot
        Trigger deployIntake = intakeButton.and(extendIntake.negate());

        // If neither the intake button or trigger are active, retract the intake
        Trigger retractIntake = intakeButton.negate().and(extendIntake.negate());

        extendIntake.whenActive(new SequentialCommandGroup(
                new ExtendIntakeVariable(intake, () -> 0.1).withTimeout(50),
                new PivotIntake(Intake.IntakeState.COLLECT, intake),
                new SetRollerState(intakeRoller, IntakeRoller.States.INTAKE),
                new ExtendIntakeVariable(intake, () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))
        ));

        deployIntake.whenActive(new SequentialCommandGroup(
                new ExtendIntakeVariable(intake, () -> 0.1).withTimeout(50), // tune this
                new PivotIntake(Intake.IntakeState.COLLECT, intake),
                new SetRollerState(intakeRoller, IntakeRoller.States.INTAKE)
        ));

        retractIntake.whenActive(new SequentialCommandGroup(
                new SetRollerState(intakeRoller, IntakeRoller.States.STOP),
                new RetractIntake(intake),
                new PivotIntake(Intake.IntakeState.HOME, intake)));

        kickerButton.whenPressed(new SetKickerPosition(false, intake))
                .whenReleased(new SetKickerPosition(true, intake));

        armButton.whenHeld(new InstantCommand(() -> arm.goToPos(Arm.ArmState.SCORE)))
                        .whenReleased(new InstantCommand(() -> arm.goToPos(Arm.ArmState.INTAKE)));

        outtakeButton.whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                    new PivotIntake(Intake.IntakeState.STORE, intake),
                    new RunCommand(() -> {}).withTimeout(300),
                    new SetRollerState(intakeRoller, IntakeRoller.States.OUTTAKE)),
                new SequentialCommandGroup(
                        new SetRollerState(intakeRoller, IntakeRoller.States.OUTTAKE)
                ),
                () -> {
                    if (intake.getState() == Intake.IntakeState.STORE || intake.getState() == Intake.IntakeState.HOME) {
                        return true;
                    } else {
                        return false;
                    }
                }))
        .whenReleased(new ConditionalCommand(
            new SequentialCommandGroup(
                new SetRollerState(intakeRoller, IntakeRoller.States.STOP),
                new PivotIntake(Intake.IntakeState.HOME, intake)),
            new SequentialCommandGroup(new SetRollerState(intakeRoller, IntakeRoller.States.INTAKE)),
            () -> {
                if (intake.getState() == Intake.IntakeState.STORE) {
                    return true;
                } else {
                    return false;
                }
            }));

        clawButton.whenPressed(new OpenClaw(claw)).whenReleased(new CloseClaw(claw));

//        ScoreAtBucket.whenPressed(new ScoreAtBucket(drivetrain, arm, elevator));
//
        elevatorUpButton.whenPressed(new ElevatorGoTo(elevator, 35));

        elevatorDownButton.whenPressed(new ElevatorGoTo(elevator, 0));

        CommandScheduler.getInstance().setDefaultCommand(elevator, new ManualElevatorCommand(elevator,
                () -> (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), telemetry));

        drivetrain.setDefaultCommand(new DefaultDrive(drivetrain,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()));

        register(arm, intake, intakeRoller);
        schedule(new RunCommand(telemetry::update));

        waitForStart();
        schedule(new PivotIntake(Intake.IntakeState.HOME, intake), new RetractIntake(intake));
    }
}
