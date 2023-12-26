package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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
    MoveArmCommand frontwardCommand, backwardCommand;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriveSubsystem");

    @Override
    public void initialize() {
        GamepadEx controller1 = new GamepadEx(gamepad1);
        GamepadEx controller2 = new GamepadEx(gamepad2);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);

        driveSubsystem = new DriveSubsystem(driveMotors);
        /*driveCommand = new DefaultDrive(driveSubsystem,
                () -> -controller1.getLeftY(),
                () -> controller1.getLeftX(),
                () -> controller1.getRightX());
        */
        driveCommand = new DefaultDrive(driveSubsystem,
                () -> -controller1.getLeftY(),
                controller1::getLeftX,
                controller1::getRightX);

        armSubsystem = new ArmSubsystem(hardwareMap, "armMotor");
        frontwardCommand = new MoveArmCommand(armSubsystem, ArmSubsystem.Direction.FRONTWARD);
        backwardCommand = new MoveArmCommand(armSubsystem, ArmSubsystem.Direction.BACKWARD);

        controller1.getGamepadButton(GamepadKeys.Button.A).whenPressed(frontwardCommand);
        controller1.getGamepadButton(GamepadKeys.Button.B).whenPressed(backwardCommand);

        dbp.createNewTelePacket();
        dbp.info("Initializing");
        dbp.send(true);

        register(armSubsystem);
        armSubsystem.setDefaultCommand(new MoveArmCommand(armSubsystem, null));
        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
