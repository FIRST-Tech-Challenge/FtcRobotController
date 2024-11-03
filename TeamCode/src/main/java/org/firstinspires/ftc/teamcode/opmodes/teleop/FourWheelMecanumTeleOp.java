package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSlider;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

/**
 * This is old mainTeleop
 */
@TeleOp (name = "main_teleop")
@SuppressWarnings("unused")
public class FourWheelMecanumTeleOp extends OpModeTemplate {

    private TeleFourWheelMecanumDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        DeliveryPivot deliveryPivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, feedback);
        DeliverySlider deliverySlider = new DeliverySlider(hardwareMap, operatorGamepad, telemetry, feedback);
        IntakeSlider intakeSlider = new IntakeSlider(hardwareMap, operatorGamepad, telemetry, feedback);
        RollingIntake rollingIntake = new RollingIntake(hardwareMap, operatorGamepad, telemetry, feedback);

        driveTrain = new TeleFourWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, feedback);

        switchToMode(PowerMode.REGULAR);

        // Delivery Pivot
        new Trigger(() -> gamepad2.right_stick_y > 0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsIntake, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        new Trigger(() -> gamepad2.right_stick_y < -0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsDelivery, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(deliveryPivot::RotateTowardsIntakeSlowly, deliveryPivot))
                .whenReleased(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(deliveryPivot::RotateTowardsDeliverySlowly, deliveryPivot))
                .whenReleased(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        // Delivery Slider
        new Trigger(() -> gamepad2.left_stick_y > 0.5)
                .whenActive(new InstantCommand(deliverySlider::Expand, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));

        new Trigger(() -> gamepad2.left_stick_y < -0.5)
                .whenActive(new InstantCommand(deliverySlider::Collapse, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(rollingIntake::Intake, rollingIntake))
                .whenReleased(new InstantCommand(rollingIntake::Hold, rollingIntake));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(rollingIntake::Outtake, rollingIntake))
                .whenReleased(new InstantCommand(rollingIntake::Hold, rollingIntake));

        // Drivetrain speed
        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToNitroMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        new Trigger(() -> gamepad1.left_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToSlowMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        // Robot direction
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));
        // servo test, for the speciman part claw thingy
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));

        // Toggle wrist angle
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(rollingIntake::SetElbowInSpecimenPosition, rollingIntake));
        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(rollingIntake::SetElbowInIntakePosition, rollingIntake));

        // Register all subsystems
        register(driveTrain, deliveryPivot, feedback, rollingIntake);

        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }

    @Override
    public void run() {
        super.run();
    }

    private void switchToNitroMode() {
        switchToMode(PowerMode.NITRO);
    }

    private void switchToRegularMode() {
        switchToMode(PowerMode.REGULAR);
    }

    private void switchToSlowMode() {
        switchToMode(PowerMode.SLOW);
    }

    private void switchToMode(PowerMode powerMode) {
        driveTrain.setPowerRatio(powerMode.getPowerRatio());
    }
}