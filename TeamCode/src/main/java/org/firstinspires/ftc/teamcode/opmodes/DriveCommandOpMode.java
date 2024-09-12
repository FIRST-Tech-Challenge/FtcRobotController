package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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

    private DefaultDrive driveCommand;


    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriverOP");

    @Override
    public void initialize() {
        driverController = new GamepadEx(gamepad1);
        armerController = new GamepadEx(gamepad2);

        dbp.createNewTelePacket();
        dbp.info("Initializing drive command op mode...");
        dbp.send(false);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);

        HashMap<RobotHardwareInitializer.Arm, DcMotor> armMotors = RobotHardwareInitializer.initializeArm(this);

        assert driveMotors != null;
        driveSubsystem = new DriveSubsystem(driveMotors);

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

        register(driveSubsystem);

        driveSubsystem.setDefaultCommand(driveCommand);

        dbp.info("Subsystems registered.");
        dbp.send(false);

        dbp.info("Ready.");
        dbp.send(false);

        waitForStart();

        dbp.info("GO GO GO!");
        dbp.send(false);
    }

    private void initializeDriveSuppliers() {
        slowdownMultiplier = () -> 1d / ((driverController.getButton(slowdownButton)
                || driverController.getButton(slowdownButton2)
                || driverController.getButton(GamepadKeys.Button.A)) ? 1d : 2d);
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
