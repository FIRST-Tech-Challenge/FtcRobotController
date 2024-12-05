package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.DumpBucketCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendoCommand;
import org.firstinspires.ftc.teamcode.commands.MovePincherCommand;
import org.firstinspires.ftc.teamcode.commands.SetUppiesCommand;
import org.firstinspires.ftc.teamcode.commands.TiltIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ToggleIntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendoSystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PincherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.UppiesSubsystem;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

import java.util.HashMap;
import java.util.Set;
import java.util.function.DoubleSupplier;

@TeleOp(name = "RealestDriverOpMode")
public class DriveCommandOpMode extends CommandOpMode {


    private GamepadEx driverController, armerController;
    private final GamepadKeys.Button slowdownButton = GamepadKeys.Button.RIGHT_BUMPER;
    private final GamepadKeys.Button slowdownButton2 = GamepadKeys.Button.LEFT_BUMPER;

    private DoubleSupplier slowdownMultiplier, forwardBack, leftRight, rotation;

    private DriveSubsystem driveSubsystem;
    private HangSubsystem hangSubsystem;
    private PincherSubsystem pincherSubsystem;
    private UppiesSubsystem uppiesSubsystem;
    private BucketSubsystem bucketSubsystem;
    private ExtendoSystem extendoSystem;
    private IntakeSubsystem intakeSubsystem;

    private DefaultDrive driveCommand;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("DriverOP");

    @Override
    public void initialize() {
        driverController = new GamepadEx(gamepad1);
        armerController = new GamepadEx(gamepad2);

        dbp.createNewTelePacket();
        dbp.info("Initializing drive command op mode...");
        dbp.send(false);

        try {
            // HashMap<RobotHardwareInitializer.Component, DcMotor> driveMotors = RobotHardwareInitializer.initializeDriveMotors(hardwareMap, this);
            // assert driveMotors != null;
            driveSubsystem = new DriveSubsystem(hardwareMap);
            register(driveSubsystem);
            initializeDriveSuppliers();
            driveCommand = new DefaultDrive(driveSubsystem, gamepad1);
            driveSubsystem.setDefaultCommand(driveCommand);
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
            return;
        }

        try {
            ServoEx pincher1 = RobotHardwareInitializer.ServoComponent.FINGER_1.getEx(hardwareMap, 0, 45);
            ServoEx pincher2 = RobotHardwareInitializer.ServoComponent.FINGER_2.getEx(hardwareMap, 0, 45);
            pincherSubsystem  = new PincherSubsystem(pincher1, pincher2);
            register(pincherSubsystem);

            armerController.getGamepadButton(GamepadKeys.Button.X).whenPressed(new MovePincherCommand(pincherSubsystem, PincherSubsystem.FingerPositions.CLOSED));
            armerController.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new MovePincherCommand(pincherSubsystem, PincherSubsystem.FingerPositions.OPEN));
            // NOT NEEDED IN GAME! This is for debug purposes only and personal testing
            //armerController.getGamepadButton(GamepadKeys.Button.X).whenPressed(new MovePincherCommand(pincherSubsystem, PincherSubsystem.FingerPositions.ZERO));
        } catch (Exception e) {
            //e.printStackTrace();
            throw new RuntimeException(e);
        }

        try {
            DcMotorEx uppiesMotor = RobotHardwareInitializer.MotorComponent.UPPIES.getEx(hardwareMap);
            uppiesSubsystem = new UppiesSubsystem(uppiesMotor);
            register(uppiesSubsystem);

            SetUppiesCommand idleCommand = new SetUppiesCommand(uppiesSubsystem, UppiesSubsystem.UppiesState.IDLE);
            SetUppiesCommand upCommand = new SetUppiesCommand(uppiesSubsystem, UppiesSubsystem.UppiesState.UPPIES);
            SetUppiesCommand downCommand = new SetUppiesCommand(uppiesSubsystem, UppiesSubsystem.UppiesState.UN_UPPIES);
            armerController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                    .whenPressed(upCommand)
                    .whenReleased(idleCommand);
            armerController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(downCommand)
                    .whenReleased(idleCommand);
        } catch (Exception e) {
            //e.printStackTrace();
            throw new RuntimeException(e);
        }

        try {
            ServoEx bucketServo = RobotHardwareInitializer.ServoComponent.BUCKET_DUMPER.getEx(hardwareMap, 0, 135);
            bucketSubsystem = new BucketSubsystem(bucketServo);
            register(bucketSubsystem);
            // R3
            armerController.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new DumpBucketCommand(bucketSubsystem));
        } catch (Exception e) {
            //e.printStackTrace();
            throw new RuntimeException(e);
        }

        try {
            DcMotorEx motorEx = RobotHardwareInitializer.MotorComponent.EXTENDER.getEx(hardwareMap);
            extendoSystem = new ExtendoSystem(motorEx);
            register(extendoSystem);

            ExtendoCommand noMovementCommand = new ExtendoCommand(extendoSystem, ExtendoSystem.Direction.NONE);
            armerController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(new ExtendoCommand(extendoSystem, ExtendoSystem.Direction.INWARD)).whenReleased(noMovementCommand);
            armerController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(new ExtendoCommand(extendoSystem, ExtendoSystem.Direction.OUTWARD)).whenReleased(noMovementCommand);
        } catch (Exception e) {
            //e.printStackTrace();
            throw new RuntimeException(e);
        }

        try {
            DcMotorEx intakePower = RobotHardwareInitializer.MotorComponent.INTAKE.getEx(hardwareMap);
            ServoEx intakeTilter = RobotHardwareInitializer.ServoComponent.INTAKE_TILTER.getEx(hardwareMap);
            intakeSubsystem = new IntakeSubsystem(intakePower, intakeTilter);

            armerController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .toggleWhenPressed(new TiltIntakeCommand(intakeSubsystem, true), new TiltIntakeCommand(intakeSubsystem, false));
            armerController.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new ToggleIntakeCommand(intakeSubsystem));
        } catch (Exception e) {
            //e.printStackTrace();
            throw new RuntimeException(e);
        }

        dbp.info("Subsystems registered.");
        dbp.send(false);

        dbp.info("Ready.");
        dbp.send(false);

        waitForStart();

        dbp.info("GO GO GO!");
        dbp.send(false);

        while (opModeIsActive()) {
            if (gamepad2.a) {
                pincherSubsystem.locomoteFinger(PincherSubsystem.FingerPositions.OPEN);
                dbp.info("OPEN PINCHER");
                dbp.send(true);
                telemetry.addData("pincher state", "true");
            }
            if (gamepad2.b) {
                pincherSubsystem.locomoteFinger(PincherSubsystem.FingerPositions.CLOSED);
                dbp.info("CLOSE PINCHER");
                dbp.send(true);
                telemetry.addData("pincher state", "false");
            }
        }

    }

    @Override
    public void reset() {
        super.reset();
        // Reset the finger back to the original position
        if (pincherSubsystem != null) {
            pincherSubsystem.locomoteFinger(PincherSubsystem.FingerPositions.ZERO);
        }
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
