package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;

/**
 * This is old mainTeleop
 */
@TeleOp
@SuppressWarnings("unused")
public class DoubleMotorSlider extends OpModeTemplate {

    private TeleFourWheelMecanumDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        DeliverySlider deliverySlider = new DeliverySlider(hardwareMap, driverGamepad, null, null);

        // Delivery Slider
        new Trigger(() -> gamepad2.left_stick_y > 0.5)
                .whenActive(new InstantCommand(deliverySlider::collapse, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));

        new Trigger(() -> gamepad2.left_stick_y < -0.5)
                .whenActive(new InstantCommand(deliverySlider::expand, deliverySlider))
                .whenInactive(new InstantCommand(deliverySlider::Hold, deliverySlider));


        // Register all subsystems
        register(deliverySlider);

    }

    @Override
    public void run() {
        super.run();
    }

}