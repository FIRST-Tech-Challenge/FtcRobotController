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
        GamepadEx driverController = new GamepadEx(gamepad1);
        GamepadEx armerController = new GamepadEx(gamepad2);

        dbp.createNewTelePacket();
        dbp.info("Initializing");
        dbp.send(true);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);

        driveSubsystem = new DriveSubsystem(driveMotors);
        armSubsystem = new ArmSubsystem(RobotHardwareInitializer.initializeArm(this));
        wristSubsystem = new WristSubsystem(RobotHardwareInitializer.initializeWrist(this));
        fingerSubsystem = new FingerSubsystem(RobotHardwareInitializer.initializeFinger(this));

        GamepadKeys.Button slowdownButton = GamepadKeys.Button.A;

        // TODO: Make the D-pad for the driver controller work like WASD
        // TODO: Check if the current D-pad implementation works as intended

        int dpadX =
                (driverController.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0) -
                        (driverController.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0);
        int dpadY =
                (driverController.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : 0) -
                        (driverController.getButton(GamepadKeys.Button.DPAD_DOWN) ? 1 : 0);

        DoubleSupplier forwardBack = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                if(dpadY != 0) {
                    return dpadY / ((driverController.getButton(slowdownButton) ? 1d : 2d));
                } else {
                    return driverController.getLeftY() / ((driverController.getButton(slowdownButton) ? 1d : 2d));
                }
            }
        };
        DoubleSupplier leftRight = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                if(dpadX != 0) {
                    return dpadX / ((driverController.getButton(slowdownButton) ? 1d : 2d));
                } else {
                    return driverController.getLeftX() / ((driverController.getButton(slowdownButton) ? 1d : 2d));
                }
            }
        };
        DoubleSupplier rotation = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return driverController.getRightX() / ((driverController.getButton(slowdownButton) ? 1d : 2d));
            }
        };

        driveCommand = new DefaultDrive(driveSubsystem,
                forwardBack,
                leftRight,
                rotation);

        // TODO: autonomous macro for arm positioning (aka bumpers automatically move the arm to the pickup position or to the board position)

        register(driveSubsystem);
        register(armSubsystem);
        register(wristSubsystem);
        register(fingerSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(new MoveArmCommand(armSubsystem,
                () -> armerController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> armerController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        wristSubsystem.setDefaultCommand(new MoveWristCommand(wristSubsystem,
                () -> (armerController.getButton(GamepadKeys.Button.DPAD_LEFT) ? 0 : 1),
                () -> (armerController.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 0 : 1)));
        fingerSubsystem.setDefaultCommand(new MoveFingerCommand(fingerSubsystem,
                () -> (armerController.getButton(GamepadKeys.Button.A) ? 0 : 1),
                () -> (armerController.getButton(GamepadKeys.Button.B) ? 0 : 1)));
    }
}
