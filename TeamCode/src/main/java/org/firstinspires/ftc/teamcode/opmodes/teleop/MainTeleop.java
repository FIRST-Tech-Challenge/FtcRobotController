package org.firstinspires.ftc.teamcode.opmodes.teleop;

import android.util.Log;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.opmodes.PowerMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.MovePivotRelativelyCommand;
import org.firstinspires.ftc.teamcode.subsystems.climb.HangingArm;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSlider;
import org.firstinspires.ftc.teamcode.subsystems.intake.MultiAxisIntake;
//import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSliderClaw;

/**
 * This is old mainTeleop
 */
@TeleOp
@SuppressWarnings("unused")
public class MainTeleop extends OpModeTemplate {

    private static final String LOG_TAG = MainTeleop.class.getSimpleName();
    private TeleDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();


        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        RollingIntake rollingIntake = new RollingIntake(hardwareMap, operatorGamepad, telemetry, feedback);
        DeliveryPivot deliveryPivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, feedback, rollingIntake);
        DeliverySlider deliverySlider = new DeliverySlider(hardwareMap, operatorGamepad, telemetry, feedback, deliveryPivot);
        deliverySlider.setPivotLowEnoughSupplier(deliveryPivot::lowEnoughToLimitSlider);
        SpecimenSlider specimenSlider = new SpecimenSlider(hardwareMap, telemetry, feedback);
        //LimeLight limeLight = new LimeLight(hardwareMap, telemetry);
        HangingArm hangingArm = new HangingArm(hardwareMap, telemetry, driverGamepad, feedback);
        SpecimenSliderClaw specimenSliderClaw = new SpecimenSliderClaw(hardwareMap, telemetry, feedback);
        MultiAxisIntake multiAxisIntake = new MultiAxisIntake(hardwareMap, operatorGamepad, telemetry, feedback);

        driveTrain = new TeleDriveTrain(hardwareMap, driverGamepad, telemetry, feedback, null);

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


        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(() -> {
            double sliderPosition = deliverySlider.getPosition();
            double pivotPosition = deliveryPivot.getPosition();
            Log.i(LOG_TAG, String.format("slider position = %f, pivot position = %f", sliderPosition, pivotPosition));

        });
        // Delivery Slider
        new Trigger(() -> gamepad2.left_stick_y > 0.5)
                .whenActive(new InstantCommand(deliverySlider::collapse, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));

        new Trigger(() -> gamepad2.left_stick_y < -0.5)
                .whenActive(new InstantCommand(deliverySlider::expand, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));


        // Intake and Outtake

        new Trigger(() -> gamepad2.right_trigger > 0.5)
                .whenActive(new InstantCommand(rollingIntake::Intake, rollingIntake))
                .whenInactive(new InstantCommand(rollingIntake::Hold, rollingIntake));

        new Trigger(() -> gamepad2.left_trigger > 0.5)
                .whenActive(new InstantCommand(rollingIntake::Outtake, rollingIntake))
                .whenInactive(new InstantCommand(rollingIntake::Hold, rollingIntake));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(rollingIntake::ToggleElbowAcrossAll, rollingIntake));

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(deliverySlider::MoveToMaxExtension, deliverySlider));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
        //        .whenPressed(new AlignToSampleUsingLimelight(driveTrain, limeLight, telemetry, 3000));

        // Specimen Slider Actions

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(specimenSlider::Expand, specimenSlider))
                .whenReleased(new InstantCommand(specimenSlider::Hold, specimenSlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(specimenSlider::Collapse, specimenSlider))
                .whenReleased(new InstantCommand(specimenSlider::Hold, specimenSlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(multiAxisIntake::setIntakeSample, multiAxisIntake));

        // Toggle wrist angle
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new SequentialCommandGroup(
                        new InstantCommand(deliverySlider::MoveToDeliverySamplePosition, deliverySlider),
                        new InstantCommand(rollingIntake::ToggleElbowPosition, rollingIntake)
                ));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new ParallelCommandGroup(
                        new InstantCommand(rollingIntake::ToggleElbowPosition, rollingIntake),
                        new InstantCommand(deliverySlider::MoveToCollapsedPosition, deliverySlider)
                ));


        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(deliveryPivot::AutoToDelivery, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(new InstantCommand(deliveryPivot::AutoToIntake, deliveryPivot));

        // Pivot back to start position - previously on operator gamepad
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(deliveryPivot::AutoToStart, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(rollingIntake::ToggleElbowPosition, rollingIntake));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(deliverySlider::ResetEncoder, deliverySlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                    .whenPressed(new MovePivotRelativelyCommand(deliveryPivot, MovePivotRelativelyCommand.Direction.ToPickup, deliveryPivot.DeliveryPositionFromStart - 400, telemetry)
                            .andThen(new InstantCommand(deliveryPivot::resetEncoder, deliveryPivot)));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        //        .whenPressed(new InstantCommand(driveTrain::AlignTx, driveTrain));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        //        .whenPressed(new InstantCommand(driveTrain::AlignTy, driveTrain));

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(new InstantCommand(hangingArm::collapse, hangingArm))
                .whenReleased(new InstantCommand(hangingArm::hold, hangingArm));

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new InstantCommand(hangingArm::extend, hangingArm))
                .whenReleased(new InstantCommand(hangingArm::hold, hangingArm));

        // DRIVER Actions

        // Drivetrain speed
        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToNitroMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        new Trigger(() -> gamepad1.left_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToSlowMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));


        // Robot direction
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));


        // servo test, for the speciman part claw thingy
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));

        // Register all subsystems
        register(driveTrain, deliveryPivot, deliverySlider, feedback, rollingIntake, specimenSlider); //, limeLight);

        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }

    @Override
    public void run() {
        super.run();
    }

    boolean fastMode = true;
    private void ToggleSpeed() {

        if(fastMode) {
            switchToMode(PowerMode.NITRO);
        } else {
            switchToMode(PowerMode.SLOW);
        }
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