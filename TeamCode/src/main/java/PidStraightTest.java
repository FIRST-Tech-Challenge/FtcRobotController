/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.teamcode.SensorBNO055IMU;

@TeleOp
public class PidTest4_24 extends LinearOpMode {
    private BNO055IMU imu;

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    // Define constants for motor encoder counts
    public static final double TICKS_PER_REVOLUTION = 537.7; // Replace with your motor's ticks per revolution
    public static final double WHEEL_DIAMETER_CM = 9.6; // Replace with your wheel diameter
    public static final double RADIUS = 1.875;

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;


    @Override
    public void runOpMode() {
        // Initialize your motors and other hardware components
        motorFrontLeft = hardwareMap.get(DcMotor.class, "lf");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rf");
        motorBackLeft = hardwareMap.get(DcMotor.class, "lr");
        motorBackRight = hardwareMap.get(DcMotor.class, "rr");


        // Reverse motors if needed
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set motor modes to run using encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);



        waitForStart();
        //Drive forward 1 tile
        //PIDStraight(100, 200);;
        pidTurn(90);
        // Stop the robot
        stopRobot();
    }


    //public void PIDStraight(double target_encoder, double timeout, double kMaxSpeed, double heading) {
    public void PIDStraight(double target_cm, double kMaxSpeed) {
        // Constants for PID control
        final double kPDrive = 1.65 ;
        final double kIDrive = 0.0;
        final double kDDrive = 1.0;
        //final double Kpturn_correction = 4;
        final double move_error = 1.0;
        int end_count = 0;


        // Variables for tracking progress
        double startTime = timer.milliseconds();
        double error_drive = 0;
        double integral_drive = 0.0;
        double derivative_drive = 0;
        double prevError_drive = 0.0;


        // Reset the drive motor encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Main PID control loop
        while (opModeIsActive() && (end_count < 1)) {
            //&& (timer.milliseconds() - startTime < timeout) && opModeIsActive()) {
            //double headingError = heading - imu.getAngularOrientation().firstAngle;
            //double headingCorrection = headingError * Kpturn_correction;


            // Calculate the error, integral, and derivative terms for PID control
            double currentTime = timer.milliseconds();
            double deltaTime = currentTime-startTime;
            prevError_drive = error_drive;
            int leftEncoderValue = (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition())/2;
            int rightEncoderValue = (motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition())/2;
            //double current_tick = (leftEncoderValue + rightEncoderValue);
            double current_tick = (leftEncoderValue + rightEncoderValue) / 2.0;
            double current_dist = (current_tick * WHEEL_DIAMETER_CM * Math.PI) / TICKS_PER_REVOLUTION;
            error_drive = target_cm - current_dist;


            // Limit the heading correction to be within the acceptable range
            //headingCorrection = Math.max(-Kpturn_correction, Math.min(headingCorrection, Kpturn_correction));


            // Calculate the integral term
            integral_drive += error_drive * deltaTime; // deltaTime is the time since the last iteration


            // Calculate the derivative term
            derivative_drive = (error_drive - prevError_drive) / deltaTime;


            // Apply heading correction to motor speeds
            double leftSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive;
            double rightSpeed = kPDrive * error_drive + kIDrive * integral_drive + kDDrive * derivative_drive ;


            // Limit the motor speeds to be within the acceptable range
            leftSpeed = Math.max(-kMaxSpeed, Math.min(leftSpeed, kMaxSpeed));
            rightSpeed = Math.max(-kMaxSpeed, Math.min(rightSpeed, kMaxSpeed));


            // Adjust for turning correction
            //leftSpeed += Kpturn_correction * (heading - imu.getAngularOrientation().firstAngle);
            //rightSpeed -= Kpturn_correction * (heading - imu.getAngularOrientation().firstAngle);


            // Set the motor speeds
            motorFrontLeft.setPower(leftSpeed);
            motorFrontRight.setPower(rightSpeed);
            motorBackLeft.setPower(leftSpeed);
            motorBackRight.setPower(rightSpeed);


            // Check if the error is within the specified move error
            if (Math.abs(error_drive) < move_error) {
                end_count++;
            } else {
                end_count = 0;
            }
            sleep(15);


            // Update telemetry or print to the screen
            telemetry.addData("Total Distance: ", current_dist);
            telemetry.update();


        }


        // Stop the motors once the target distance has been reached
        stopRobot();
    }
    public void PIDStrafe(double target_cm) {
        // Constants for PID control
        final double kPStrafe = 1.65;
        final double kIStrafe = 0.0;
        final double kDStrafe = 0.0;
        final double move_error = 1.0;
        int end_count = 0;

        // Variables for tracking progress
        double startTime = timer.milliseconds();
        double error_strafe = 0;
        double integral_strafe = 0.0;
        double derivative_strafe = 0;
        double prevError_strafe = 0.0;

        // Reset the drive motor encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to run using encoders
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Main PID control loop
        while (opModeIsActive() && (end_count < 1)) {
            // Calculate the error, integral, and derivative terms for PID control
            double currentTime = timer.milliseconds();
            double deltaTime = currentTime - startTime;
            prevError_strafe = error_strafe;

            int frontLeftEncoderValue = motorFrontLeft.getCurrentPosition();
            int frontRightEncoderValue = motorFrontRight.getCurrentPosition();
            int backLeftEncoderValue = motorBackLeft.getCurrentPosition();
            int backRightEncoderValue = motorBackRight.getCurrentPosition();

            // Calculate strafe distance for each side considering reverse movement
            double strafeLeftDistance = (frontLeftEncoderValue - backLeftEncoderValue) / 2.0;
            double strafeRightDistance = (frontRightEncoderValue - backRightEncoderValue) / 2.0;
            double strafe_cm = (strafeLeftDistance + strafeRightDistance) * WHEEL_DIAMETER_CM * Math.PI / TICKS_PER_REVOLUTION;
            error_strafe = target_cm - strafe_cm;

            // Calculate the integral term
            integral_strafe += error_strafe * deltaTime;

            // Calculate the derivative term
            derivative_strafe = (error_strafe - prevError_strafe) / deltaTime;

            // Apply strafing correction to motor speeds
            double leftSpeed = kPStrafe * error_strafe + kIStrafe * integral_strafe + kDStrafe * derivative_strafe;
            double rightSpeed = kPStrafe * error_strafe + kIStrafe * integral_strafe + kDStrafe * derivative_strafe;

            // Set the motor speeds with reversal for one side
            motorFrontLeft.setPower(leftSpeed);
            motorFrontRight.setPower(-rightSpeed);
            motorBackLeft.setPower(-leftSpeed);
            motorBackRight.setPower(rightSpeed);

            // Check if the error is within the specified move error
            if (Math.abs(error_strafe) < move_error) {
                end_count++;
            } else {
                end_count = 0;
            }
            sleep(15);

            // Update telemetry or print to the screen
            telemetry.addData("Strafe Distance: ", strafe_cm);
            telemetry.update();
        }



    // Stop the motors once the target distance has been reached
        stopRobot();
    }

    public void stopRobot() {
        // Stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        sleep(100);
    }


    public void pidTurn(double targetAngle) {
        double Kp = 2.39; // Proportional gain
        double Ki = 0.0; // Integral gain
        double Kd = 0.0; // Derivative gain
        double minSpeed = 10; // Minimum speed
        double error = 0, prevError = 0.0, integral = 0.0, derivative, output = 0;

        double startTime = timer.milliseconds();
        int endCount = 0;

        while ((endCount < 2 || Math.abs(error) > Math.abs(prevError)) && opModeIsActive()) {
            error = targetAngle - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double currentTime = timer.milliseconds();
            double deltaTime = currentTime - startTime;
            integral += error * deltaTime;
            derivative = (error - prevError) / deltaTime;

            if (Math.abs(error) < 0.7) {
                endCount++;
            } else {
                endCount = 0;
            }

            double proportional = Kp * error;

            if (prevError * error < 0) {
                integral = 0;
            }

            if (Math.abs(output) < 45) {
                integral += Ki * error;
            }

            output = proportional + integral + derivative * Kd;

            motorFrontRight.setPower(output);
            motorBackRight.setPower(output);
            motorFrontLeft.setPower(-output);
            motorBackLeft.setPower(-output);

            sleep(15);

            prevError = error;
        }

        stopRobot();
    }


}*/

