package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public interface FTC {





    @TeleOp(name = "Basic Motor Tutorial", group = "Tutorial")
    public class MotorTutorial extends LinearOpMode {

        // 1. Declare the motor object
        private DcMotor myMotor;

        @Override
        public void runOpMode() throws InterruptedException {

            // 2. Map the hardware using the exact name from the configuration app
            myMotor = hardwareMap.get(DcMotor.class, "test_motor");

            // 3. Optional Setup: Set behavior when power is set to 0
            // BRAKE stops the motor instantly; FLOAT lets it coast down smoothly.
            myMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // 4. Optional Setup: Reverse the direction if it spins backwards
            // myMotor.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("Status", "Initialized. Ready to start!");
            telemetry.update();

            // Wait for the driver to press the PLAY button on the Driver Station
            waitForStart();

            // Loop runs continuously until the STOP button is pressed
            while (opModeIsActive()) {

                // Proportional Joystick Control: Map left stick Y-axis (-1.0 to 1.0)
                // Note: Gamepad Y-axis values are natively inverted (up is negative),
                // so we add a negative sign to make pushing up equal positive power.
                double motorPower = -gamepad1.left_stick_y;

                // Send the power level to the motor
                myMotor.setPower(motorPower);

                // Send feedback to the Driver Station screen
                telemetry.addData("Motor Power Input", motorPower);
                telemetry.addData("Motor Target Power", myMotor.getPower());
                telemetry.update();
            }
        }
    }

}

