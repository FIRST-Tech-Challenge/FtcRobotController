package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
//import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

/**
 * This is old mainTeleop
 */
@TeleOp
@SuppressWarnings("unused")
public class MainTeleop extends OpModeTemplate {

    private TeleFourWheelMecanumDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        RollingIntake rollingIntake = new RollingIntake(hardwareMap, operatorGamepad, telemetry, feedback);
        DeliveryPivot deliveryPivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, feedback, rollingIntake);
        DeliverySlider deliverySlider = new DeliverySlider(hardwareMap, operatorGamepad, telemetry, feedback);
        deliverySlider.setPivotLowEnoughSupplier(deliveryPivot::lowEnoughToLimitSlider);
        //LimeLight limeLight = new LimeLight(hardwareMap, telemetry);
        //HangingArm hangingArm = new HangingArm(hardwareMap, telemetry, driverGamepad, feedback);

        driveTrain = new TeleFourWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, feedback, null);

        switchToMode(PowerMode.REGULAR);

        // OPERATOR Actions
        //stopper
        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(deliveryPivot::ToggleStopper, deliveryPivot));

        //operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
        //        .whenPressed(new InstantCommand(stopper::setPositionOpened, stopper))
        //        .whenReleased(new InstantCommand(stopper::setPositionMid, stopper));

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

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(deliverySlider::CollapseSlowly, deliverySlider))
                .whenReleased(new InstantCommand(deliverySlider::Hold, deliverySlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(deliverySlider::ExpandSlowly, deliverySlider))
                .whenReleased(new InstantCommand(deliverySlider::Hold, deliverySlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).toggleWhenPressed(
                new InstantCommand(() -> {
                    // first set of button binding
                }),
                new InstantCommand(() -> {
                    // second set of button binding
                })
        );
        // Delivery Slider
        new Trigger(() -> gamepad2.left_stick_y > 0.5)
                .whenActive(new InstantCommand(deliverySlider::Expand, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));

        new Trigger(() -> gamepad2.left_stick_y < -0.5)
                .whenActive(new InstantCommand(deliverySlider::Collapse, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));

        // Intake and Outtake

        new Trigger(() -> gamepad2.right_trigger > 0.5)
                .whenActive(new InstantCommand(rollingIntake::Intake, rollingIntake))
                .whenInactive(new InstantCommand(rollingIntake::Hold, rollingIntake));

        new Trigger(() -> gamepad2.left_trigger > 0.5)
                .whenActive(new InstantCommand(rollingIntake::Outtake, rollingIntake))
                .whenInactive(new InstantCommand(rollingIntake::Hold, rollingIntake));

        // Toggle wrist angle
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(deliverySlider::MoveToDeliveryPosition, deliverySlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new InstantCommand(deliverySlider::MoveToCollapsedPosition, deliverySlider));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(deliveryPivot::AutoToDelivery, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(new InstantCommand(deliveryPivot::AutoToIntake, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ParallelCommandGroup(
                                new InstantCommand(deliveryPivot::AutoToStart, deliveryPivot),
                                new InstantCommand(deliverySlider::MoveToDeliveryPosition, deliverySlider),
                                new InstantCommand(rollingIntake::SetElbowInSpecimenPosition, rollingIntake)
                ));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(rollingIntake::ToggleElbowPosition, rollingIntake));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(deliverySlider::ResetEncoder, deliverySlider));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
        //        .whenPressed(new InstantCommand(driveTrain::AlignTx, driveTrain));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
        //        .whenPressed(new InstantCommand(driveTrain::AlignTy, driveTrain));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
        //        .whenHeld(new InstantCommand(hangingArm::collapse, hangingArm))
        //        .whenReleased(new InstantCommand(hangingArm::hold, hangingArm));

        //driverGamepad.getGamepadButton(GamepadKeys.Button.X)
        //        .whenHeld(new InstantCommand(hangingArm::extend, hangingArm))
        //        .whenReleased(new InstantCommand(hangingArm::hold, hangingArm));
        // DRIVER Actions

        // Drivetrain speed
        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToNitroMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        new Trigger(() -> gamepad1.left_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToSlowMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        // Robot direction
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));

        // servo test, for the speciman part claw thingy
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));

        // Register all subsystems
        register(driveTrain, deliveryPivot, deliverySlider, feedback, rollingIntake);//, limeLight);

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