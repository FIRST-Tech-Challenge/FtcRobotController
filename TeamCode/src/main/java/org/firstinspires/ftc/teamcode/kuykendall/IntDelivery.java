package org.firstinspires.ftc.teamcode.kuykendall;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="IntDelivery", group="Test")
public class IntDelivery extends OpMode {

    private Servo rightInt, leftInt;
    private final double pickupPosition = .67; // Adjust this within 0 to 1
    private final double dropoffPosition = .2; // Adjust this within 0 to 1

    @Override
    public void init() {
        rightInt = hardwareMap.get(Servo.class, "rightInt");
        leftInt = hardwareMap.get(Servo.class, "leftInt");

        // Initialize servos to pickup position
        rightInt.setPosition(pickupPosition);
        leftInt.setPosition(1.0 - pickupPosition); // Assuming servos are mirrored
    }

    @Override
    public void loop() {
        // Move to 'dropoff' position when left bumper is pressed
        if (gamepad1.right_bumper) {
            rightInt.setPosition(dropoffPosition);
            leftInt.setPosition(1.0 - dropoffPosition); // Adjust for mirrored servo
        }

        // Move to 'pickup' position when right bumper is pressed
        if (gamepad1.left_bumper) {
            rightInt.setPosition(pickupPosition);
            leftInt.setPosition(1.0 - pickupPosition); // Adjust for mirrored servo
        }

        // Add telemetry data to debug
        telemetry.addData("Right Servo Position", rightInt.getPosition());
        telemetry.addData("Left Servo Position", leftInt.getPosition());
        telemetry.update();
    }
}
