package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Arm Control", group="TeleOp")
@Disabled
public class LinearSlideTest extends OpMode {

    // Define the motor for the linear slide
    private DcMotor linearSlideMotor;
    private DcMotor ArmMotor;

    // Motor power settings
    private static final double SLIDE_POWER = 0.8;   // Adjust based on required speed

    @Override
    public void init() {
        // Initialize the linear slide motor from the hardware map
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Set motor direction if necessary (adjust based on your setup)
        linearSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to brake so it holds position when stopped
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        // Check D-pad controls on the gamepad
        if (gamepad1.dpad_right) {
            // Move the slide up
            linearSlideMotor.setPower(SLIDE_POWER);

        } else if (gamepad1.dpad_left) {
            // Move the slide down
            linearSlideMotor.setPower(-SLIDE_POWER);
        } else {
            // Stop the slide if no D-pad buttons are pressed
            linearSlideMotor.setPower(0);
        }

        if (gamepad1.dpad_up) {
            // Move the slide up
            ArmMotor.setPower(SLIDE_POWER);
        } else if (gamepad1.dpad_down) {
            // Move the slide down
            ArmMotor.setPower(-SLIDE_POWER);
        } else {
            // Stop the slide if no D-pad buttons are pressed
            ArmMotor.setPower(0);
        }


        // Telemetry for monitoring
        telemetry.addData("Slide Power", linearSlideMotor.getPower());
        telemetry.addData("Arm Power", ArmMotor.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure motor is stopped when the OpMode ends
        linearSlideMotor.setPower(0);
        ArmMotor.setPower(0);
    }
}
/*

package org.firstinspires.ftc.team26396;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Arm Control", group="TeleOp")
public class LinearSlideTest extends OpMode {

    // Define the motor for the linear slide
    private DcMotor linearSlideMotor;
    private DcMotor ArmMotor;

    // Motor power settings
    private static final double SLIDE_POWER = 0.8;   // Adjust based on required speed

    @Override
    public void init() {
        // Initialize the linear slide motor from the hardware map
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        // Set motor direction if necessary (adjust based on your setup)
        linearSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set zero power behavior to brake so it holds position when stopped
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the ArmMotor's encoder position to 0
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset encoder, setting position to 0
        ArmMotor.setTargetPosition(0);                             // Set initial target to position 0
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {
        // Control to return ArmMotor to position 0 when the 'A' button is pressed
        if (gamepad1.a) {
            ArmMotor.setTargetPosition(0);     // Set target to position 0
            ArmMotor.setPower(0.5);            // Adjust power for controlled movement
        }

        // Manual control for linearSlideMotor
        if (gamepad1.dpad_up) {
            linearSlideMotor.setPower(SLIDE_POWER);
        } else if (gamepad1.dpad_down) {
            linearSlideMotor.setPower(-SLIDE_POWER);
        } else {
            linearSlideMotor.setPower(0);
        }

        // Manual control for ArmMotor
        if (gamepad1.dpad_right) {
            ArmMotor.setPower(SLIDE_POWER);
        } else if (gamepad1.dpad_left) {
            ArmMotor.setPower(-SLIDE_POWER);
        } else  {  // Only stop ArmMotor if 'A' is not pressed
            ArmMotor.setPower(0);
        }

        // Telemetry for monitoring
        telemetry.addData("Slide Power", linearSlideMotor.getPower());
        telemetry.addData("Arm Power", ArmMotor.getPower());
        telemetry.addData("Arm Position", ArmMotor.getCurrentPosition()); // Display current encoder position
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure motor is stopped when the OpMode ends
        linearSlideMotor.setPower(0);
        ArmMotor.setPower(0);
    }
}
*/