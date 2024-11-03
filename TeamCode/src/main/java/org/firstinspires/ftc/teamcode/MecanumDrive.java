package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

class MecanumDrive {
    // Getter methods to access motor positions
    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }

    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Constructor to initialize all four drive motors
    public MecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        System.out.println("Mecanum drive initialized");
    }

    // Method to drive the robot using mecanum wheels
    public void drive(double forward, double strafe, double rotate) {
        // Calculate power for each wheel based on forward, strafe, and rotate inputs
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        // Normalize the wheel powers if any power is greater than 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set power to each motor without changing signs.
        frontLeft.setPower(-frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(-backLeftPower);
        backRight.setPower(backRightPower);
        System.out.println(String.format("Drive command - FrontLeft: %2.2f, FrontRight: %2.2f, BackLeft: %2.2f, BackRight: %2.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower));
    }

    // Method to reset encoders and set motors to the desired mode
    public void resetAndRunToPosition() {
        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Method to set target position for all motors and then set mode to RUN_TO_POSITION
    public void setTargetPositionAndRun(int ticks) {
        // Set target position
        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        // Set motors to RUN_TO_POSITION mode after setting the target
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Method to set target position for all motors


    // Method to check if all motors are busy
    public boolean isBusy() {
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }
}