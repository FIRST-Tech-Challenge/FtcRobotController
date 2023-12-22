package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

@TeleOp(name = "REALDriverOperationMode")
public class DriveCommandOpMode extends CommandOpMode {

    DriveSubsystem driveSubsystem;
    DoubleSupplier axial, lateral, yaw;
    DefaultDrive driveCommand;

    @Override
    public void initialize() {
        GamepadEx driverController = new GamepadEx(gamepad1);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);
        driveSubsystem = new DriveSubsystem(driveMotors);
        axial = () -> -driverController.getLeftY();
        lateral = () -> driverController.getLeftX();
        yaw = () -> driverController.getRightX();
        driveCommand = new DefaultDrive(driveSubsystem, axial, lateral, yaw);

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
