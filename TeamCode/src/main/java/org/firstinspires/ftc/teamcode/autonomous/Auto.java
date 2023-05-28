package org.firstinspires.ftc.teamcode.autonomous;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.GoForward;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Command-based Autonomous Sample")
public class Auto extends CommandOpMode {

    static final double WHEEL_DIAMETER = 75.000104; // millimeters

    private MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;

    DriveSubsystem m_drive;
    private DefaultDrive m_driveCommand;

    @Override
    public void initialize() {

        m_frontLeft = new MotorEx(hardwareMap, "FL");
        m_frontRight = new MotorEx(hardwareMap, "FRandOdo");
        m_backLeft = new MotorEx(hardwareMap, "BLandOdo");
        m_backRight = new MotorEx(hardwareMap, "BRandOdo");

        m_drive = new DriveSubsystem(m_frontLeft, m_frontRight, m_backLeft, m_backRight, WHEEL_DIAMETER);

        m_driveCommand = new DefaultDrive(m_drive, () -> -1, () -> 0, () -> 0);

        register(m_drive);
        m_drive.setDefaultCommand(m_driveCommand);

    }
}