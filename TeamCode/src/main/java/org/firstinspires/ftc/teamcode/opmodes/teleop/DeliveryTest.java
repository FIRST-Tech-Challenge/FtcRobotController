package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

@TeleOp
public class DeliveryTest extends OpModeTemplate {

    @Override
    public void initialize() {
        super.initialize();

        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);

        DeliveryPivot deliveryPivot = new DeliveryPivot(hardwareMap, operatorGamepad, telemetry, feedback);

        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsDelivery, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        new Trigger(() -> gamepad1.left_trigger > 0.5)
                .whenActive(new InstantCommand(deliveryPivot::RotateTowardsIntake, deliveryPivot))
                .whenInactive(new InstantCommand(deliveryPivot::HoldArm, deliveryPivot));

        // Register all subsystems
        register(deliveryPivot, feedback);

        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));
    }

    @Override
    public void run() {
        super.run();
    }
}