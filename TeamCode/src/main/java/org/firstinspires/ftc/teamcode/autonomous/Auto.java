package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.old.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.GoForward;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Command-based Autonomous Sample")
public class Auto extends CommandOpMode {

    static final double WHEEL_DIAMETER = 75.000104; // millimeters

    private MotorEx m_frontLeft, m_frontRight, m_backLeft, m_backRight;

    DriveSubsystem driveSubsystem;

    @Override
    public void initialize() {

    }

    @Override
    public void run() {
        new GoForward(driveSubsystem);
    }
}