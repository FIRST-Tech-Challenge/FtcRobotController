package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="SingleOdometerTest")
public class SingleOdometerTest extends LinearOpMode {
    // Declare the encoder
    DcMotor odometryWheel;

    @Override
    public void runOpMode() {
        // Initialize the encoder - change "odometry_wheel" to match your configuration name
        odometryWheel = hardwareMap.dcMotor.get("odometry_wheel");

        // Reset encoder value
        odometryWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set encoder to read mode
        odometryWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Read and display the encoder value
            telemetry.addData("Encoder Counts", odometryWheel.getCurrentPosition());
            telemetry.update();
        }
    }
}