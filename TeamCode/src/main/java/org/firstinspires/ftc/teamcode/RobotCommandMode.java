package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;

@TeleOp
public class RobotCommandMode extends CommandOpMode {
    GamepadEx driverOp = new GamepadEx(gamepad1);
    @Override
    public void initialize() {
        HashMap<String, DcMotor> map = initializeDriveMotors(hardwareMap, this);
        schedule(new DefaultDrive(new DriveSubsystem(map.get("leftFrontDrive"),
                map.get("rightFrontDrive"), map.get("leftBackDrive"), map.get("rightBackDrive"), hardwareMap)));

    }

    @Override
    public void run() {
        super.run();
    }
}
