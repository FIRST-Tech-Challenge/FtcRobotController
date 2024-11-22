package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveWithGamepadCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorRetractCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberDropCommand;
import org.firstinspires.ftc.teamcode.commands.GrabberPickupCommand;
import org.firstinspires.ftc.teamcode.commands.WormLowerCommand;
import org.firstinspires.ftc.teamcode.commands.WormRaiseCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
        drive.setDefaultCommand(
                new DriveWithGamepadCommand(gamepad1, drive)
        );

        driver = new GamepadEx(gamepad1);
//        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(new RunCommand(elevator::extend, elevator)) ;
//        driver.getGamepadButton(GamepadKeys.Button.B).whileHeld(new RunCommand(elevator::retract, elevator));

        driver.getGamepadButton(GamepadKeys.Button.A).whileHeld(new ElevatorExtendCommand(elevator)) ;
        driver.getGamepadButton(GamepadKeys.Button.B).whileHeld(new ElevatorRetractCommand(elevator));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(new WormLowerCommand(worm,1));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new WormRaiseCommand(worm, 1));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileHeld(new GrabberPickupCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(new GrabberDropCommand(grabber));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new RunCommand(wrist::addFifteen, wrist));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new RunCommand(wrist::subFifteen, wrist));

        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
        elevator.SetWormAngle(worm.getAngle()); //set this continually so elevator can know how far it can go
    }
}
