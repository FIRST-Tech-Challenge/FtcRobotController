package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.GateCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.MoveArmCommand;
import org.firstinspires.ftc.teamcode.commands.MoveFingerCommand;
import org.firstinspires.ftc.teamcode.commands.MoveWristCommand;
import org.firstinspires.ftc.teamcode.commands.ThrowAirplaneCommand;
import org.firstinspires.ftc.teamcode.commands.WristPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ZeroWristPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
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
    private GateSubsystem gateSubsystem;


    private DefaultDrive driveCommand;
    private MoveArmCommand moveArmCommand;
    private MoveWristCommand moveWristCommand;
    private WristPositionCommand zeroWristCommand;
    private MoveFingerCommand moveFingerCommand;
    private IntakeCommand intakeCommand;


    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriverOP");

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
        wristSubsystem = new WristSubsystem(RobotHardwareInitializer.initializeWrist(this), false);
        fingerSubsystem = new FingerSubsystem(RobotHardwareInitializer.initializeFinger(this));
        launcherSubsystem = new LauncherSubsystem(RobotHardwareInitializer.initializeLauncher(this));
        gateSubsystem = new GateSubsystem(RobotHardwareInitializer.initializeGateServos(this));

        dbp.info("Subsystems built.");
        dbp.send(false);

        initializeDriveSuppliers();

        // Initialize commands for the subsystems
        DoubleSupplier forwardWristSupplier = () -> {
            double end = 0;
            end += armerController.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0;
            double leftX = armerController.getLeftX();
            if (leftX > 0) {
                end += leftX;
            }
            end = Math.max(0, Math.min(1, end));
            return end;
        };
        DoubleSupplier backwardWristSupplier = () -> {
            double end = 0;
            end += armerController.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0;
            double leftX = armerController.getLeftX();
            if (leftX < 0) {
                end -= leftX;
            }
            end = Math.max(0, Math.min(1, end));
            return end;
        };



        driveCommand = new DefaultDrive(driveSubsystem, forwardBack, leftRight, rotation);
        moveArmCommand = new MoveArmCommand(armSubsystem,
                () -> armerController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> armerController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        moveWristCommand = new MoveWristCommand(wristSubsystem,
                backwardWristSupplier, forwardWristSupplier);
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
        armerController.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new ThrowAirplaneCommand(launcherSubsystem));
        armerController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new GateCommand(gateSubsystem, GateSubsystem.GateState.OPEN));
        armerController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new GateCommand(gateSubsystem, GateSubsystem.GateState.CLOSED));
        armerController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new WristPositionCommand(wristSubsystem, true, moveWristCommand));
        armerController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new WristPositionCommand(wristSubsystem, false, moveWristCommand));
        armerController.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new ZeroWristPositionCommand(wristSubsystem));


        register(driveSubsystem);
        register(armSubsystem);
        register(wristSubsystem);
        register(fingerSubsystem);
        register(launcherSubsystem);
        register(gateSubsystem);

        driveSubsystem.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(moveArmCommand);
        wristSubsystem.setDefaultCommand(moveWristCommand);
        fingerSubsystem.setDefaultCommand(moveFingerCommand);

        dbp.info("Subsystems registered.");
        dbp.send(false);

        dbp.info("Ready.");
        dbp.send(false);

        waitForStart();

        dbp.info("GO GO GO!");
        dbp.send(false);
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
