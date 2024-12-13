package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("TeleOP");

    @Override
    public void initialize() {
        driverController = new GamepadEx(gamepad1);
        armerController = new GamepadEx(gamepad2);

        // CONTROL CONFIG
        GamepadKeys.Button closePincherButton       = GamepadKeys.Button.X;
        GamepadKeys.Button openPincherButton        = GamepadKeys.Button.Y;
        GamepadKeys.Button moveUppiesUpButton       = GamepadKeys.Button.DPAD_UP;
        GamepadKeys.Button moveUppiesDownButton     = GamepadKeys.Button.DPAD_DOWN;
        GamepadKeys.Button dumpBucketButton         = GamepadKeys.Button.RIGHT_STICK_BUTTON;
        GamepadKeys.Button retractIntakeButton      = GamepadKeys.Button.DPAD_LEFT;
        GamepadKeys.Button extendIntakeButton       = GamepadKeys.Button.DPAD_RIGHT;
        GamepadKeys.Button toggleIntakeSpinButton   = GamepadKeys.Button.LEFT_STICK_BUTTON;
        GamepadKeys.Button toggleIntakeTiltButton   = GamepadKeys.Button.LEFT_BUMPER;
        GamepadKeys.Button manualSafetyOverride     = GamepadKeys.Button.BACK;
        GamepadKeys.Button maxExtensionOverride     = GamepadKeys.Button.BACK;

        dbp.createNewTelePacket();
        dbp.info("Initializing drive command op mode...");
        dbp.send(false);

        try {
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
            ServoEx pincher1 = RobotHardwareInitializer.ServoComponent.FINGER_1.getEx(hardwareMap, 0, PincherSubsystem.MAX_ANGLE);
            ServoEx pincher2 = RobotHardwareInitializer.ServoComponent.FINGER_2.getEx(hardwareMap, 0, PincherSubsystem.MAX_ANGLE);
            pincherSubsystem  = new PincherSubsystem(pincher1, pincher2);
            register(pincherSubsystem);

            MovePincherCommand closePincher = new MovePincherCommand(pincherSubsystem, PincherSubsystem.FingerPositions.CLOSED);
            MovePincherCommand openPincher = new MovePincherCommand(pincherSubsystem, PincherSubsystem.FingerPositions.OPEN);

            armerController.getGamepadButton(closePincherButton).whenPressed(closePincher);
            armerController.getGamepadButton(openPincherButton).whenPressed(openPincher);
        } catch (Exception e) {
            dbp.info("ERROR IN PINCHER SYSTEM");
            dbp.error(e);
            dbp.send(true);
            throw new RuntimeException(e);
        }

        try {
            DcMotorEx uppiesMotor = RobotHardwareInitializer.MotorComponent.UPPIES.getEx(hardwareMap);
            uppiesSubsystem = new UppiesSubsystem(uppiesMotor);
            register(uppiesSubsystem);

            SetUppiesCommand idleCommand = new SetUppiesCommand(uppiesSubsystem, UppiesSubsystem.UppiesState.IDLE);
            SetUppiesCommand upCommand = new SetUppiesCommand(uppiesSubsystem, UppiesSubsystem.UppiesState.UPPIES);
            SetUppiesCommand downCommand = new SetUppiesCommand(uppiesSubsystem, UppiesSubsystem.UppiesState.UN_UPPIES);
            armerController.getGamepadButton(moveUppiesUpButton)
                    .whenPressed(upCommand)
                    .whenReleased(idleCommand);
            armerController.getGamepadButton(moveUppiesDownButton)
                    .whenPressed(downCommand)
                    .whenReleased(idleCommand);
        } catch (Exception e) {
            dbp.info("ERROR IN UPPIES SYSTEM");
            dbp.error(e);
            dbp.send(true);
            throw new RuntimeException(e);
        }

        try {
            ServoEx bucketServo = RobotHardwareInitializer.ServoComponent.BUCKET_DUMPER.getEx(hardwareMap, 0, 135);
            bucketSubsystem = new BucketSubsystem(bucketServo);
            register(bucketSubsystem);

            DumpBucketCommand dumpBucketCommand = new DumpBucketCommand(bucketSubsystem);

            armerController.getGamepadButton(dumpBucketButton).whenPressed(dumpBucketCommand);
        } catch (Exception e) {
            dbp.info("ERROR IN BUCKET SYSTEM");
            dbp.error(e);
            dbp.send(true);
            throw new RuntimeException(e);
        }

        try {
            DcMotorEx motorEx = RobotHardwareInitializer.MotorComponent.EXTENDER.getEx(hardwareMap);
            DcMotorEx motorReverseEx = RobotHardwareInitializer.MotorComponent.EXTENDER2.getEx(hardwareMap);
            extendoSystem = new ExtendoSystem(motorEx, motorReverseEx);
            register(extendoSystem);

            ExtendoCommand extendCommand = new ExtendoCommand(extendoSystem, ExtendoSystem.Direction.OUTWARD);
            ExtendoCommand retractCommand = new ExtendoCommand(extendoSystem, ExtendoSystem.Direction.INWARD);
            ExtendoCommand idleCommand = new ExtendoCommand(extendoSystem, ExtendoSystem.Direction.NONE);

            armerController.getGamepadButton(retractIntakeButton)
                    .whileHeld(retractCommand)
                    .whenReleased(idleCommand);
            armerController.getGamepadButton(extendIntakeButton)
                    .whileHeld(extendCommand)
                    .whenReleased(idleCommand);
        } catch (Exception e) {
            dbp.info("ERROR IN EXTENDING SYSTEM");
            dbp.error(e);
            dbp.send(true);
            throw new RuntimeException(e);
        }

        try {
            DcMotorEx intakePower = RobotHardwareInitializer.MotorComponent.INTAKE.getEx(hardwareMap);
            ServoEx intakeTilter = RobotHardwareInitializer.ServoComponent.INTAKE_TILTER.getEx(hardwareMap);
            intakeSubsystem = new IntakeSubsystem(intakePower, intakeTilter);

            TiltIntakeCommand tiltCommand = new TiltIntakeCommand(intakeSubsystem, true);
            TiltIntakeCommand untiltCommand = new TiltIntakeCommand(intakeSubsystem, false);
            ToggleIntakeCommand toggleIntakeState = new ToggleIntakeCommand(intakeSubsystem);

            armerController.getGamepadButton(toggleIntakeTiltButton)
                    .toggleWhenPressed(tiltCommand, untiltCommand);
            armerController.getGamepadButton(toggleIntakeSpinButton)
                    .whenPressed(toggleIntakeState);
            armerController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(() -> {
                intakeSubsystem.setIntakeDirection(DcMotorSimple.Direction.REVERSE);
            }, () -> {
                intakeSubsystem.setIntakeDirection(DcMotorSimple.Direction.FORWARD);
            });
        } catch (Exception e) {
            dbp.info("ERROR IN INTAKE SYSTEM");
            dbp.error(e);
            dbp.send(true);
            throw new RuntimeException(e);
        }

        dbp.info("Subsystems registered.");
        dbp.send(false);

        dbp.info("Ready.");
        dbp.send(false);

        waitForStart();

        dbp.info("GO GO GO!");
        dbp.send(false);

        /*armerController.getGamepadButton(manualSafetyOverride).toggleWhenPressed(() -> {
            UppiesSubsystem.PROGRAMATIC_STALL_SAFETY = false;
        }, () -> {
            UppiesSubsystem.PROGRAMATIC_STALL_SAFETY = true;
        });*/
        armerController.getGamepadButton(maxExtensionOverride).toggleWhenPressed(() -> {
            UppiesSubsystem.PROGRAMATIC_IGNORE_LIMITS = false;
        }, () -> {
            UppiesSubsystem.PROGRAMATIC_IGNORE_LIMITS = true;
        });

        // NOTE: Do not include the opModeIsActive() while loop, as it prevents commands from running
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
            if (dpadY != 0) {
                return dpadY * slowdownMultiplier.getAsDouble();
            } else {
                return driverController.getLeftY() * slowdownMultiplier.getAsDouble();
            }
        };
        leftRight = () -> {
            int dpadX = (driverController.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : 0)
                    - (driverController.getButton(GamepadKeys.Button.DPAD_LEFT) ? 1 : 0);
            if (dpadX != 0) {
                return dpadX * slowdownMultiplier.getAsDouble();
            } else {
                return driverController.getLeftX() * slowdownMultiplier.getAsDouble();
            }
        };
    }

}
