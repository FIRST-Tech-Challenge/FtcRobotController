package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.commands.MoveFingerCommand;
import org.firstinspires.ftc.teamcode.commands.MoveWristCommand;
import org.firstinspires.ftc.teamcode.commands.ThrowAirplaneCommand;
import org.firstinspires.ftc.teamcode.commands.ZeroWristCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.DoubleSupplier;

@TeleOp(name = "RealestDriverOpMode")
public class DriveCommandOpMode extends CommandOpMode {

    private GamepadEx driverController, armerController;
    private final GamepadKeys.Button slowdownButton = GamepadKeys.Button.RIGHT_BUMPER;
    private final GamepadKeys.Button slowdownButton2 = GamepadKeys.Button.LEFT_BUMPER;

    private DoubleSupplier slowdownMultiplier, forwardBack, leftRight, rotation;

    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private FingerSubsystem fingerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LauncherSubsystem launcherSubsystem;

    private DefaultDrive driveCommand;
    private MoveArmCommand moveArmCommand;
    private MoveWristCommand moveWristCommand;
    private ZeroWristCommand zeroWristCommand;
    private MoveFingerCommand moveFingerCommand;
    private IntakeCommand intakeCommand;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriveSubsystem");

    @Override
    public void initialize() {
        driverController = new GamepadEx(gamepad1);
        armerController = new GamepadEx(gamepad2);

        dbp.createNewTelePacket();
        dbp.info("Initializing drive command op mode...");
        dbp.send(false);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);

        assert driveMotors != null;
        driveSubsystem = new DriveSubsystem(driveMotors);
        armSubsystem = new ArmSubsystem(RobotHardwareInitializer.initializeArm(this));
        wristSubsystem = new WristSubsystem(RobotHardwareInitializer.initializeWrist(this));
        fingerSubsystem = new FingerSubsystem(RobotHardwareInitializer.initializeFinger(this));
        intakeSubsystem = new IntakeSubsystem(RobotHardwareInitializer.initializeIntake(this));
        launcherSubsystem = new LauncherSubsystem(RobotHardwareInitializer.initializeLauncher(this));

        dbp.info("Subsystems built.");
        dbp.send(false);

        initializeDriveSuppliers();

        // Initialize commands for the subsystems

        driveCommand = new DefaultDrive(driveSubsystem, forwardBack, leftRight, rotation);
        moveArmCommand = new MoveArmCommand(armSubsystem,
                () -> armerController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> armerController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> armerController.getButton(GamepadKeys.Button.LEFT_BUMPER),
                () -> armerController.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        moveWristCommand = new MoveWristCommand(wristSubsystem,
                () -> (armerController.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0) -
                        (armerController.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0));
        moveFingerCommand = new MoveFingerCommand(fingerSubsystem,
                () -> {
                    double quantity = (armerController.getButton(GamepadKeys.Button.A) ? 1 : 0) +
                            (armerController.getButton(GamepadKeys.Button.X) ? 1 : 0);
                    if(quantity == 0) {
                        return 0;
                    }
                    return quantity/2.5d;
                },
                () -> {
                    double quantity = (armerController.getButton(GamepadKeys.Button.B) ? 1 : 0) +
                            (armerController.getButton(GamepadKeys.Button.Y) ? 1 : 0);
                    if(quantity == 0) {
                        return 0;
                    }
                    return quantity/2.5d;
                });
        intakeCommand = new IntakeCommand(intakeSubsystem);
        armerController.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new ThrowAirplaneCommand(launcherSubsystem));
        armerController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ZeroWristCommand(wristSubsystem, () -> (armerController.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0)
                        - (armerController.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0), () -> 0));

        register(driveSubsystem);
        register(armSubsystem);
        register(wristSubsystem);
        register(fingerSubsystem);
        register(intakeSubsystem);
        register(launcherSubsystem);

        driveSubsystem.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(moveArmCommand);
        wristSubsystem.setDefaultCommand(moveWristCommand);
        fingerSubsystem.setDefaultCommand(moveFingerCommand);
        intakeSubsystem.setDefaultCommand(intakeCommand);

        dbp.info("Subsystems registered.");
        dbp.send(false);

        dbp.info("Ready.");
        dbp.send(false);

        waitForStart();
        schedule(intakeCommand);
    }

    private void initializeDriveSuppliers() {
        slowdownMultiplier = () -> 1d / ((driverController.getButton(slowdownButton) || driverController.getButton(slowdownButton2) || driverController.getButton(GamepadKeys.Button.A)) ? 1d : 2d);
        rotation = () -> driverController.getRightX() * slowdownMultiplier.getAsDouble();

        forwardBack = () -> {
            int dpadY = (driverController.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : 0)
                    - (driverController.getButton(GamepadKeys.Button.DPAD_DOWN) ? 1 : 0);
            if(dpadY != 0) {
                return dpadY * slowdownMultiplier.getAsDouble();
            } else {
                return driverController.getLeftY() * slowdownMultiplier.getAsDouble();
            }
        };
        leftRight = () -> {
            int dpadX = (driverController.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0)
                    - (driverController.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0);
            if(dpadX != 0) {
                return dpadX * slowdownMultiplier.getAsDouble();
            } else {
                return driverController.getLeftX() * slowdownMultiplier.getAsDouble();
            }
        };
    }

}
