package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
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
import org.firstinspires.ftc.teamcode.subsystems.EmergencyArmSubsystem;
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
    private EmergencyArmSubsystem armSubsystem;
    private PincherSubsystem pincherSubsystem;

    private DefaultDrive driveCommand;

    private final FTCDashboardPackets dbp = new FTCDashboardPackets("TeleOP");

    @Override
    public void initialize() {
        driverController = new GamepadEx(gamepad1);
        armerController = new GamepadEx(gamepad2);

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
            hangSubsystem = new HangSubsystem(hardwareMap);

            // D-Pad Up and D-Pad Down toggles the manages the hanging

            armerController.getGamepadButton(GamepadKeys.Button.Y).whileActiveContinuous(() -> {
                hangSubsystem.setHangDirection(HangSubsystem.HangDirection.UP);
            }).whenInactive(() -> {
                hangSubsystem.setHangDirection(HangSubsystem.HangDirection.IDLE);
            });

            armerController.getGamepadButton(GamepadKeys.Button.X).whileActiveContinuous(() -> {
                hangSubsystem.setHangDirection(HangSubsystem.HangDirection.DOWN);
            }).whenInactive(() -> {
                hangSubsystem.setHangDirection(HangSubsystem.HangDirection.IDLE);
            });
        } catch (Exception e) {
            dbp.info("ERROR IN HANG SYSTEM");
            dbp.error(e);
            dbp.send(true);
            telemetry.addData("Hang", "Error in hang subsystem: "+e.getMessage());
            telemetry.update();
            throw new RuntimeException(e);
        }

        try {
            armSubsystem = new EmergencyArmSubsystem(hardwareMap, telemetry);
            float threshold = .1f;



            // Bumpers handle the lower arm
            // Triggers handle the higher arm
            // "A" toggles the pincher state
            // Left Joystick X moves the wrist

            DoubleSupplier higherSupplier = () -> armerController.getRightY();//() -> armerController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - armerController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
            DoubleSupplier lowerSupplier = () -> armerController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - armerController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            new Trigger(() -> Math.abs(higherSupplier.getAsDouble()) > threshold).whileActiveContinuous(() -> {
                armSubsystem.setHigherArmPower(higherSupplier.getAsDouble());
            }).whenInactive(() -> {
                armSubsystem.setHigherArmPower(0);
            });

            new Trigger(() -> Math.abs(lowerSupplier.getAsDouble()) > threshold).whileActiveContinuous(() -> {
                armSubsystem.setLowerArmPower(lowerSupplier.getAsDouble());
            }).whenInactive(() -> {
                armSubsystem.setLowerArmPower(0);
            });

            /*armerController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(() -> {
                armSubsystem.setLowerArmPower(-1);
            }).whenInactive(() -> {
                 armSubsystem.setLowerArmPower(0);
            });

            armerController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(() -> {
                armSubsystem.setLowerArmPower(1);
            }).whenInactive(() -> {
                armSubsystem.setLowerArmPower(0);
            });*/

            /*armerController.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(() -> {
                armSubsystem.setPinchState(EmergencyArmSubsystem.PinchState.OPEN);
            }, () -> armSubsystem.setPinchState(EmergencyArmSubsystem.PinchState.PINCHED));*/

            new Trigger(() -> Math.abs(armerController.getLeftX()) > threshold).whileActiveContinuous(() -> {
                armSubsystem.setWristPower(armerController.getLeftX());
            }).whenInactive(() -> armSubsystem.setWristPower(0));

            final double angularVelocityLower = 25;
            armerController.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
                armSubsystem.constantX(angularVelocityLower);
            }).whenInactive(() -> {
                armSubsystem.setLowerArmPower(0);
                armSubsystem.setHigherArmPower(0);
            });
            armerController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
                armSubsystem.constantX(-angularVelocityLower);
            }).whenInactive(() -> {
                armSubsystem.setLowerArmPower(0);
                armSubsystem.setHigherArmPower(0);
            });

            armerController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
                armSubsystem.constantY(angularVelocityLower); // Todo: Might need to swap the negative sign
            }).whenInactive(() -> {
                armSubsystem.setLowerArmPower(0);
                armSubsystem.setHigherArmPower(0);
            });
            armerController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
                armSubsystem.constantY(-angularVelocityLower); // Todo: Might need to swap the negative sign
            }).whenInactive(() -> {
                armSubsystem.setLowerArmPower(0);
                armSubsystem.setHigherArmPower(0);
            });

            /*armerController.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(() -> {
                armSubsystem.setPinchState(EmergencyArmSubsystem.PinchState.OPEN);
            }, () -> {
                armSubsystem.setPinchState(EmergencyArmSubsystem.PinchState.PINCHED);
            });*/

            new Trigger(() -> Math.abs(armerController.getLeftX()) > threshold).whileActiveContinuous(() -> {
                armSubsystem.setWristPower(armerController.getLeftX());
            }).whenInactive(() -> {
                armSubsystem.setWristPower(0);
            });
        } catch (Exception e) {
            dbp.info("ERROR IN ARM SYSTEM");
            dbp.error(e);
            dbp.send(true);
            telemetry.addData("Arm", "Error in arm subsystem: "+e.getMessage());
            telemetry.update();
            throw new RuntimeException(e);
        }

        try {
            ServoEx pincher1 = RobotHardwareInitializer.ServoComponent.FINGER_1.getEx(hardwareMap, 0, PincherSubsystem.MAX_ANGLE);
            ServoEx pincher2 = RobotHardwareInitializer.ServoComponent.FINGER_2.getEx(hardwareMap, 0, PincherSubsystem.MAX_ANGLE);
            pincherSubsystem  = new PincherSubsystem(pincher1, pincher2);
            register(pincherSubsystem);

            MovePincherCommand closePincher = new MovePincherCommand(pincherSubsystem, PincherSubsystem.FingerPositions.CLOSED);
            MovePincherCommand openPincher = new MovePincherCommand(pincherSubsystem, PincherSubsystem.FingerPositions.OPEN);

            armerController.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(closePincher, openPincher);
            pincherSubsystem.closeFinger();
        } catch (Exception e) {
            dbp.info("ERROR IN PINCHER SYSTEM");
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

        // NOTE: Do not include the opModeIsActive() while loop, as it prevents commands from running
    }

    @Override
    public void reset() {
        super.reset();
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
