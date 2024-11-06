package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

// Define the OpMode name and type
@Disabled
public class RobotAutonomous extends LinearOpMode {

    // Declare motors for robot propulsion
    private DcMotor leftMotor, rightMotor;

    // Define the encoder ticks per revolution (TPR) for the motors
    private static final double COUNTS_PER_REV = 1120; // Change this value according to your motor's specification
    private static final double WHEEL_DIAMETER_INCHES = 4; // Change this value according to your wheel's specification

    // Define the distance calculation variables
    private static final double DRIVE_GEAR_REDUCTION = 1.0; // Gear ratio of motor
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES; // Wheel circumference in inches
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE; // Counts per inch

    @Override
    public void runOpMode() {

        // Initialize the motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Reverse the direction of the right motor if needed
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the start button to be pressed
        waitForStart();

        // Call the parking method to perform autonomous parking
        park();

        // Stop the motors once the parking is complete
        stopMotors();
    }

    private void park() {

        // Drive the robot forward for a specified distance (e.g., 12 inches)
        driveForward(12);

        // Turn the robot to face the backstage area
        turnRight(90);

        // Drive the robot forward to park into the backstage area
        driveForward(24);
    }

    private void driveForward(double inches) {

        // Calculate the target position for the motor encoders
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        // Set the target position for both motors
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        // Set the mode of motors to RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the robot's power
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        // Wait until both motors have reached their target position
        while (opModeIsActive() && (leftMotor.isBusy() || rightMotor.isBusy())) {
            // Continue execution
            idle();
        }

        // Stop the motors
        stopMotors();
    }

    private void turnRight(double degrees) {
        // Similar to driveForward method, implement the turning logic for the desired degrees
        // Make use of the encoder counts to determine the target position
        // Adjust motor speed and other parameters as necessary
    }

    private void stopMotors() {
        // Stop both motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set the motors' mode to STOP_AND_RESET_ENCODER
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}