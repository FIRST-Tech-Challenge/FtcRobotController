package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;

@TeleOp
public class Calibration extends OpModeTemplate {

    @Override
    public void initialize() {
        super.initialize();

        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        RollingIntake rollingIntake = new RollingIntake(hardwareMap, operatorGamepad, telemetry, feedback);
        DeliveryPivot deliveryPivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, feedback, rollingIntake);
        DeliverySlider deliverySlider = new DeliverySlider(hardwareMap, operatorGamepad, telemetry, feedback);

        TeleFourWheelMecanumDriveTrain driveTrain = new TeleFourWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, feedback, null);

        new Trigger(() -> gamepad2.right_stick_y > 0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsDelivery, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        new Trigger(() -> gamepad2.right_stick_y < -0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsIntake, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        new Trigger(() -> gamepad2.left_stick_y > 0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsDeliverySlowly, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        new Trigger(() -> gamepad2.left_stick_y < -0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsIntakeSlowly, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(deliveryPivot::Calibrate, deliveryPivot));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(rollingIntake::SetElbowInSampleDeliveryPosition, rollingIntake));

        // Register all subsystems
        register(deliveryPivot, deliverySlider, driveTrain, rollingIntake, feedback);

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
    }

    @Override
    public void run() {
        super.run();
    }
}