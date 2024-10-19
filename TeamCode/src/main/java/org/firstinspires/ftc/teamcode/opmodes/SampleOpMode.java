package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.ExtendIntake;
import org.firstinspires.ftc.teamcode.commands.ManualElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/***
 * Sample OpMode that uses the CommandOpMode base class.
 *
 */
@TeleOp
public class SampleOpMode extends CommandOpMode {
    // GamepadEx is a better version of the gamepad object you are used to.
    // You can setup command to be ran on button press, release, or hold
    // you also don't have to invert the value from the triggers and sticks.
    private GamepadEx driver;
    private GamepadEx operator;

    private Arm arm;
    private Intake intake;
    private Elevator elevator;
    private Drivetrain drivetrain;
    @Override
    public void initialize() {
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        arm = new Arm(this.hardwareMap);
        intake = new Intake(this.hardwareMap);
        elevator = new Elevator(this.hardwareMap, telemetry);
        drivetrain = new Drivetrain(this.hardwareMap, telemetry);


        GamepadButton armButton = new GamepadButton(
                operator, GamepadKeys.Button.A
        );

        GamepadButton intakeButton = new GamepadButton(
                operator, GamepadKeys.Button.B
        );

        GamepadButton outtakeButton = new GamepadButton(
                operator, GamepadKeys.Button.X
        );

        GamepadButton elevatorUpButton = new GamepadButton(
                operator, GamepadKeys.Button.LEFT_BUMPER
        );

        GamepadButton elevatorDownButton = new GamepadButton(
                operator, GamepadKeys.Button.RIGHT_BUMPER
        );

        armButton.whenHeld(new InstantCommand(() -> arm.goToPos(Arm.ArmState.SCORE)))
                        .whenReleased(new InstantCommand(() -> arm.goToPos(Arm.ArmState.INTAKE)));

        intakeButton.whenHeld(new ExtendIntake(intake).andThen(
                new PivotIntake(Intake.IntakeState.COLLECT,intake).andThen(
                        new InstantCommand(() -> intake.rollerIntake()))))
       .whenReleased(new InstantCommand(() -> intake.rollerStop()).andThen(
               new PivotIntake(Intake.IntakeState.STORE, intake).andThen(
                       new RetractIntake(intake))));

        outtakeButton.whenHeld(new InstantCommand(() -> intake.rollerOuttake()))
                        .whenReleased(new InstantCommand(() -> intake.rollerStop()));

        elevatorUpButton.whenPressed(new ElevatorGoTo(elevator, 20));

        elevatorDownButton.whenPressed(new ElevatorGoTo(elevator, 0));

        CommandScheduler.getInstance().setDefaultCommand(elevator, new ManualElevatorCommand(elevator,
                () -> (operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), telemetry));

        CommandScheduler.getInstance().setDefaultCommand(drivetrain, new DefaultDrive(drivetrain,
                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX()));

        register(arm, intake);
        schedule(new RunCommand(telemetry::update));

        waitForStart();
        schedule(new PivotIntake(Intake.IntakeState.HOME, intake));
    }
}
