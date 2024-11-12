package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.MoveBucketArmCommand;
import org.firstinspires.ftc.teamcode.commands.MoveFingerCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
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
    private HangSubsystem hangSubsystem;
    private FingerSubsystem fingerSubsystem;

    private DefaultDrive driveCommand;
    private MoveBucketArmCommand bucketCommand;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriverOP");

    @Override
    public void initialize() {
        driverController = new GamepadEx(gamepad1);
        armerController = new GamepadEx(gamepad2);

        dbp.createNewTelePacket();
        dbp.info("Initializing drive command op mode...");
        dbp.send(false);

        HashMap<RobotHardwareInitializer.DriveMotor, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);

        HashMap<RobotHardwareInitializer.Arm, DynamicTypeValue> armMotors = RobotHardwareInitializer.initializeArm(this);

        ServoEx finger1 = hardwareMap.get(ServoEx.class, "finger1");
        ServoEx finger2 = hardwareMap.get(ServoEx.class, "finger2");

        assert driveMotors != null;
        driveSubsystem = new DriveSubsystem(driveMotors);

        dbp.info("Subsystems built.");
        dbp.send(false);

        initializeDriveSuppliers();

        driveCommand = new DefaultDrive(driveSubsystem, forwardBack, leftRight, rotation);
        armSubsystem = new ArmSubsystem(armMotors);
        fingerSubsystem = new FingerSubsystem(finger1, finger2);

        bucketCommand = new MoveBucketArmCommand(armSubsystem,
                () -> driverController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                () -> driverController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        register(driveSubsystem);
        register(armSubsystem);
        register(fingerSubsystem);

        driveSubsystem.setDefaultCommand(driveCommand);
        armSubsystem.setDefaultCommand(bucketCommand);

        armerController.getGamepadButton(GamepadKeys.Button.A).whenPressed(new MoveFingerCommand(fingerSubsystem, FingerSubsystem.FingerPositions.CLOSED));
        armerController.getGamepadButton(GamepadKeys.Button.B).whenPressed(new MoveFingerCommand(fingerSubsystem, FingerSubsystem.FingerPositions.OPEN));
        // NOT NEEDED IN GAME! This is for debug purposes only and personal testing
        armerController.getGamepadButton(GamepadKeys.Button.X).whenPressed(new MoveFingerCommand(fingerSubsystem, FingerSubsystem.FingerPositions.ZERO));


        dbp.info("Subsystems registered.");
        dbp.send(false);

        dbp.info("Ready.");
        dbp.send(false);

        waitForStart();

        dbp.info("GO GO GO!");
        dbp.send(false);

    }

    @Override
    public void reset() {
        super.reset();
        // Reset the finger back to the original position
        fingerSubsystem.locomoteFinger(FingerSubsystem.FingerPositions.ZERO);
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
