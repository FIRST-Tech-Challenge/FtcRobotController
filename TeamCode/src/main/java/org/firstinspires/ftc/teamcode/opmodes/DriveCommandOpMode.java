package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

@TeleOp(name = "REALDriverOperationMode")
public class DriveCommandOpMode extends CommandOpMode {

    DriveSubsystem driveSubsystem;
    DefaultDrive driveCommand;

    ArmSubsystem armSubsystem;

    @Override
    public void initialize() {
        GamepadEx driverController = new GamepadEx(gamepad1);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);
        driveSubsystem = new DriveSubsystem(driveMotors);
        driveCommand = new DefaultDrive(driveSubsystem,
                () -> -driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX());

        register(driveSubsystem);
        schedule(driveCommand);
        //driveSubsystem.setDefaultCommand(driveCommand);
    }
}
