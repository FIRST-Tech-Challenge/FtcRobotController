package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Dhruv_Switch_Test", group = "TeleOp")
public class Dhruv_Switch_Test extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor touch;
    DcMotor motor;

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        TouchSensor limit = hardwareMap.get(TouchSensor.class, "Limit");
        motor = hardwareMap.get(DcMotor.class, "Motor");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the Magnetic Limit Switch is pressed, stop the motor
            if (limit.isPressed()) {
                motor.setPower(0);
            } else { // Otherwise, run the motor
                motor.setPower(0.3);
            }

            telemetry.addData("Arm Motor Power:", motor.getPower());
            telemetry.update();
        }
    }
}