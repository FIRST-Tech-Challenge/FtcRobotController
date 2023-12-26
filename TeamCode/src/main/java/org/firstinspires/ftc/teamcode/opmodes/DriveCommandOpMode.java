package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.commands.MoveFingerCommand;
import org.firstinspires.ftc.teamcode.commands.MoveWristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

@TeleOp(name = "REALDriverOperationMode")
public class DriveCommandOpMode extends CommandOpMode {

    DriveSubsystem driveSubsystem;
    DoubleSupplier axial, lateral, yaw;
    DefaultDrive driveCommand;

    ArmSubsystem armSubsystem;
    WristSubsystem wristSubsystem;
    FingerSubsystem fingerSubsystem;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriveSubsystem");

    @Override
    public void initialize() {
        GamepadEx controller1 = new GamepadEx(gamepad1);
        GamepadEx controller2 = new GamepadEx(gamepad2);

        dbp.createNewTelePacket();
        dbp.info("Initializing");
        dbp.send(true);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);

        driveSubsystem = new DriveSubsystem(driveMotors);
        armSubsystem = new ArmSubsystem(RobotHardwareInitializer.initializeArm(this));
        wristSubsystem = new WristSubsystem(RobotHardwareInitializer.initializeWrist(this));
        fingerSubsystem = new FingerSubsystem(RobotHardwareInitializer.initializeFinger(this));

        driveCommand = new DefaultDrive(driveSubsystem,
                controller1::getLeftY,
                controller1::getLeftX,
                controller1::getRightX);


        register(driveSubsystem);
        register(armSubsystem);
        register(wristSubsystem);
        register(fingerSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(new MoveArmCommand(armSubsystem,
                () -> controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        wristSubsystem.setDefaultCommand(new MoveWristCommand(wristSubsystem,
                () -> (controller1.getButton(GamepadKeys.Button.DPAD_UP) ? 0 : 1),
                () -> (controller1.getButton(GamepadKeys.Button.DPAD_DOWN) ? 0 : 1)));
        fingerSubsystem.setDefaultCommand(new MoveFingerCommand(fingerSubsystem,
                () -> (controller1.getButton(GamepadKeys.Button.A) ? 0 : 1),
                () -> (controller1.getButton(GamepadKeys.Button.B) ? 0 : 1)));
    }
}
