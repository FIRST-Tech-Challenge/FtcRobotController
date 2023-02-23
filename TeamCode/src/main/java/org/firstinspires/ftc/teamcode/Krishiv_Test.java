package org.firstinspires.ftc.teamcode;// Import the necessary classes
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Krishiv_Test", group = "TeleOp")

public class Krishiv_Test extends OpMode {

    // Declare the motor objects
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Declare constants for the encoder counts per revolution and the wheel diameter
    private static final int ENCODER_COUNTS_PER_REV = 1120;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;

    // Initialize the hardware
    public void init() {

        // Retrieve the motor objects from the hardware map
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // Set the mode of the motors to RUN_USING_ENCODER
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse the direction of the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Loop and print the encoder values
    public void loop() {
        // Calculate the distance traveled by each wheel
        double leftDistance = (leftMotor.getCurrentPosition() / (double)ENCODER_COUNTS_PER_REV) * WHEEL_DIAMETER_INCHES * Math.PI;
        double rightDistance = (rightMotor.getCurrentPosition() / (double)ENCODER_COUNTS_PER_REV) * WHEEL_DIAMETER_INCHES * Math.PI;

        // Print the encoder values and distances traveled
        telemetry.addData("Left Encoder Value", leftMotor.getCurrentPosition());
        telemetry.addData("Right Encoder Value", rightMotor.getCurrentPosition());
        telemetry.addData("Left Distance", leftDistance);
        telemetry.addData("Right Distance", rightDistance);
        telemetry.update();
    }
}
