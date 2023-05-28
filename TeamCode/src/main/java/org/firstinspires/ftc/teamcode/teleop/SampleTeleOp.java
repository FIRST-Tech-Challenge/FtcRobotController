package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "Sample TeleOp")
public class SampleTeleOp extends CommandOpMode {

    static final double WHEEL_DIAMETER = 75.000104; // millimeters

    private MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private DriveSubsystem m_drive;
    private GamepadEx m_driverOp;
    private DefaultDrive m_driveCommand;

    @Override
    public void initialize() {
        m_frontLeft = new MotorEx(hardwareMap, "FL");
        m_frontRight = new MotorEx(hardwareMap, "FRandOdo");
        m_backLeft = new MotorEx(hardwareMap, "BLandOdo");
        m_backRight = new MotorEx(hardwareMap, "BRandOdo");

        m_drive = new DriveSubsystem(m_frontLeft, m_frontRight, m_backLeft, m_backRight, WHEEL_DIAMETER);

        m_driverOp = new GamepadEx(gamepad1);
        m_driveCommand = new DefaultDrive(m_drive, () -> -m_driverOp.getLeftY(), () -> -m_driverOp.getRightX(), () -> -m_driverOp.getLeftX());

        register(m_drive);
        m_drive.setDefaultCommand(m_driveCommand);

    }

}