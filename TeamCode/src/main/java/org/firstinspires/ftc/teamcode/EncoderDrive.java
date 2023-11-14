package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous
public class EncoderDrive extends LinearOpMode {

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    BNO055IMU imu;

    // Define constants for motor encoder counts
    public static final int TICKS_PER_REVOLUTION = 1440; // Replace with your motor's ticks per revolution
    public static final double WHEEL_DIAMETER_INCHES = 4.0; // Replace with your wheel diameter
    public static final double DRIVE_SPEED = 0.5; // Adjust the speed as needed
    public static final int TARGET_DISTANCE_INCHES = 12; // Replace with your target distance
    public static final double RADIUS = 1.875;

    double integralSum = 0;
    double Kp = 0.5; //Currently Testing
    double Ki = 0;
    double Kd = 0;
    double Kf = 0; //no clue what this is

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    @Override
    public void runOpMode() {
        // Initialize your motors and other hardware components
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        // Reverse motors if needed
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes to run using encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        // Drive forward 1 tile
        PIDStraight(1440, 5000, 100, 0);;


        // Stop the robot
        stopRobot();
    }

    public void PIDStraight(double target_encoder, double timeout, double kMaxSpeed, double heading) {
        // Initialize hardware components
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Set motor run mode for PID control
        DcMotor.RunMode originalRunMode = motorFrontLeft.getMode();
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Constants for PID control
        final double kPDrive = 0.68;
        final double kIDrive = 0.005;
        final double kDDrive = 0.0;
        final double Kpturn_correction = 4;
        final double move_error = 1.5;
        int end_count = 0;

        // Variables for tracking progress
        double startTime = timer.milliseconds();
        double error_drive = 0.0; //initialized
        double integral_drive = 0.0;
        double derivative_drive = 0.0; //initialized
        double prevError_drive = 0.0;

        // Reset the drive motor encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the target position for the motors
        int targetPosition = (int) (TICKS_PER_REVOLUTION * target_encoder / (Math.PI * WHEEL_DIAMETER_INCHES)); // ticks per inch
        motorFrontLeft.setTargetPosition(targetPosition);
        motorBackLeft.setTargetPosition(targetPosition);
        motorFrontRight.setTargetPosition(targetPosition);
        motorBackRight.setTargetPosition(targetPosition);

        // Enable RUN_TO_POSITION mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        motorFrontLeft.setPower(kMaxSpeed);
        motorFrontRight.setPower(kMaxSpeed);
        motorBackRight.setPower(kMaxSpeed);
        motorBackLeft.setPower(kMaxSpeed);

        // Main PID control loop
        while ((end_count < 2) && (timer.milliseconds() - startTime < timeout) && opModeIsActive()) {
            double headingError = heading - imu.getAngularOrientation().firstAngle;
            double headingCorrection = headingError * Kpturn_correction;

            // Calculate the error, integral, and derivative terms for PID control
            prevError_drive = error_drive;
            int leftEncoderValue = motorFrontLeft.getCurrentPosition();
            int rightEncoderValue = motorFrontRight.getCurrentPosition();
            double current_dist = (leftEncoderValue + rightEncoderValue) / 2.0;
            error_drive = targetPosition - current_dist;

            // Limit the heading correction to be within the acceptable range
            headingCorrection = Math.max(-Kpturn_correction, Math.min(headingCorrection, Kpturn_correction));

            // Apply heading correction to motor speeds
            double leftSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive + headingCorrection;
            double rightSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive - headingCorrection;

            // Limit the motor speeds to be within the acceptable range
            leftSpeed = Math.max(-kMaxSpeed, Math.min(leftSpeed, kMaxSpeed));
            rightSpeed = Math.max(-kMaxSpeed, Math.min(rightSpeed, kMaxSpeed));

            // Adjust for turning correction
            leftSpeed += Kpturn_correction * (heading - imu.getAngularOrientation().firstAngle);
            rightSpeed -= Kpturn_correction * (heading - imu.getAngularOrientation().firstAngle);

            // Set the motor speeds
            motorFrontLeft.setPower(leftSpeed);
            motorFrontRight.setPower(leftSpeed);
            motorBackLeft.setPower(rightSpeed);
            motorBackRight.setPower(rightSpeed);

            // Check if the error is within the specified move error
            if (Math.abs(error_drive) < move_error) {
                end_count++;
            } else {
                end_count = 0;
            }

            // Update telemetry or print to the screen
            telemetry.addData("Total Distance: ", current_dist / (TICKS_PER_REVOLUTION * target_encoder / (Math.PI * WHEEL_DIAMETER_INCHES)));
            telemetry.update();
        }

        // Stop the motors once the target distance has been reached
        stopRobot();

        // Restore the original motor run mode
        motorFrontLeft.setMode(originalRunMode);
        motorFrontRight.setMode(originalRunMode);
        motorBackRight.setMode(originalRunMode);
        motorBackLeft.setMode(originalRunMode);
    }

    public void stopRobot() {
        // Stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void pidTurn(double targetAngle, double timeout) {
        // Initialize hardware components
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Define constants for PID algorithm
        // double Kp = 0.0012 * error + 2.39; <-- not needed
        double Kp = 2.39; // <-- seems large af my friend :)
        double Ki = 0.02;
        double Kd = 0.0;
        double minspeed = 10;

        // Reset the PID variables
        double error = 0.0;
        double prevError = 0.0;
        double integral = 0.0;
        double derivative = 0.0;
        double output = 0.0;

        // Initialize time-related variables
        double startTime = timer.milliseconds();
        int end_count = 0;

        // Loop until the robot reaches the target angle
        while ((end_count < 2 || Math.abs(error) > Math.abs(prevError)) && opModeIsActive()) {
            // Update prevError
            prevError = error;

            // Calculate error
            double elapsedTime = timer.milliseconds() - startTime;
            if (elapsedTime > timeout * 1000) {
                break;
            }

            error = targetAngle - imu.getAngularOrientation().firstAngle;

            if (Math.abs(error) < 0.7) {
                end_count += 1;
            } else {
                end_count = 0;
            }

            // Calculate the proportional component of the PID algorithm
            double proportional = Kp * error;

            // Calculate the integral component of the PID algorithm
            if (prevError * error < 0) {
                integral = 0;
            }

            // Calculate the derivative component of the PID algorithm
            derivative = error - prevError;

            if (Math.abs(output) < 45) {
                integral += Ki * error;
            }

            // Calculate the output of the PID algorithm
            output = proportional + integral + derivative * Kd;

            // Set the motor speeds based on the output
            motorFrontLeft.setPower(output);
            motorFrontRight.setPower(output);
            motorBackLeft.setPower(-output);
            motorBackRight.setPower(-output);

            // Wait a short time before looping again
            sleep(20);
        }

        // Stop the motors when the target is reached
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
}