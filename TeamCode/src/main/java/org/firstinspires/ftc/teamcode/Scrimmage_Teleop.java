package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.commands.WristUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Worm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@TeleOp(name="Scrimmage: TeleOp", group="Scrimmage")
public class Scrimmage_Teleop extends CommandOpMode {

    private Elevator elevator;
    private Worm worm;
    private Grabber grabber;
    private Wrist wrist;
    private Drive drive;
    private GamepadEx driver;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("intialized", "true");
        elevator = new Elevator(hardwareMap, telemetry);
        worm = new Worm(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);

        drive = new Drive(hardwareMap, telemetry);

        driver = new GamepadEx(gamepad1);

        configV2();

        telemetry.update();
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
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new ElevatorExtendCommand(elevator));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristUpCommand(wrist),
                        new WaitCommand(100)
                )
        );

        new Trigger(() -> driver.getLeftX() >  0.5).whileActiveContinuous(
                new SequentialCommandGroup(
                        new WristDownCommand(wrist),
                        new WaitCommand(100)
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenActive(new GrabberPickupToggleCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.X).toggleWhenActive(new GrabberDropToggleCommand(grabber));
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
        elevator.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go

        boolean slowDown = driver.getButton(GamepadKeys.Button.Y);

        drive.arcadeDrive(slowDown ? driver.getRightY() * 0.5 : driver.getRightY(), slowDown ? driver.getRightX() * 0.5 : driver.getRightX(), driver.getButton(GamepadKeys.Button.B), false);

        worm.setPower(driver.getLeftY());
    }
}
