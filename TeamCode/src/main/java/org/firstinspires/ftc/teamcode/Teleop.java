package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ElevatorExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorRetractCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberDropCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberDropToggleCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberPickupCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberPickupToggleCommand;
import org.firstinspires.ftc.teamcode.commands.WormLowerCommand;
import org.firstinspires.ftc.teamcode.commands.WormRaiseCommand;
import org.firstinspires.ftc.teamcode.commands.WristDownCommand;
import org.firstinspires.ftc.teamcode.commands.WristStopCommand;
import org.firstinspires.ftc.teamcode.commands.WristUpCommand;
import org.firstinspires.ftc.teamcode.commands.ClimberDownCommand;
import org.firstinspires.ftc.teamcode.commands.ClimberStopCommand;
import org.firstinspires.ftc.teamcode.commands.ClimberUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.Climber;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@TeleOp(name="TeleOp", group="000Real")
public class Teleop extends CommandOpMode {

    private Elevator elevator;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private Drive drive;
    private GamepadEx driver;
    private Climber climber;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
        elevator = new Elevator(hardwareMap, telemetry);
        worm = new Worm(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry, true);

        climber = new Climber(hardwareMap, telemetry);
        climber.Goto(0);

        drive = new Drive(hardwareMap, telemetry);

        driver = new GamepadEx(gamepad1);

        configV2();


    }

    public void configV1() {
        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(new ElevatorExtendCommand(elevator)) ;
        driver.getGamepadButton(GamepadKeys.Button.B).whileHeld(new ElevatorRetractCommand(elevator));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new WormLowerCommand(worm,1));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new WormRaiseCommand(worm, 1));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new GrabberPickupCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new GrabberDropCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new RunCommand(wrist::addFifteen, wrist));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new RunCommand(wrist::subFifteen, wrist));
    }

    public void configV2() {
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new ElevatorExtendCommand(elevator));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new ElevatorRetractCommand(elevator));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristUpCommand(wrist)
                ), true
        ).whenInactive(
                new WristStopCommand(wrist)
        );

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >  0.5).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristDownCommand(wrist)
                ), true
        ).whenInactive(
                new WristStopCommand(wrist)
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileActiveContinuous(
                new SequentialCommandGroup(
                        new ClimberUpCommand(climber)
                ), true
        ).whenInactive(
                new ClimberStopCommand(climber)
        );

        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >  0.5).whileActiveContinuous(
                new ParallelCommandGroup(
                        new ClimberDownCommand(climber)
                ), true
        ).whenInactive(
                new ClimberStopCommand(climber)
        );


        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(new GrabberPickupToggleCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.B).toggleWhenActive(new GrabberDropToggleCommand(grabber));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();

        elevator.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go

        //if the worm state starts to lower and it hits the horizontal extension, let's pull it back, unless the elevator is currently moving
        if (worm.CurrentState == Worm.WormState.Lowering && elevator.getHorizontalExtension() >= 19 && elevator.Status == Elevator.ElevatorStatus.Stopped) {
            telemetry.addData("Attempting to retract Elevator", "true");
            elevator.retractTwoInches();
        }

        boolean slowDown = driver.getButton(GamepadKeys.Button.Y);

        drive.arcadeDrive(slowDown ? driver.getLeftY() * 0.5 : driver.getLeftY(), slowDown ? driver.getLeftX() * 0.5 : driver.getLeftX(), driver.getButton(GamepadKeys.Button.X), false);

        //invert the power to match the up and down motion
        worm.setPower(-driver.getRightY());
    }
}
