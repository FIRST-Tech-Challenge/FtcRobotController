package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivechain {

    static DcMotor frontLeft;
    static DcMotor frontRight;
    static DcMotor backLeft;
    static DcMotor backRight;

    public static void driveChainInit(DcMotor frontLeftWheel, DcMotor frontRightWheel, DcMotor backLeftWheel, DcMotor backRightWheel) {
        frontLeft = frontLeftWheel;
        frontRight = frontRightWheel;
        backLeft = backLeftWheel;
        backRight = backRightWheel;
    }

    public static void drive(double leftStick, double rightStick) {
        double y = -leftStick; // Remember, this is reversed!
        double x = rightStick * 1.1; // Counteract imperfect strafing
        double rx = rightStick;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) * 0.65/ denominator;
        double backLeftPower = (y - x + rx) * 0.65/ denominator;
        double frontRightPower = (y - x - rx) * 0.65/ denominator;
        double backRightPower = (y + x - rx)* 0.65/ denominator;
        frontLeft.setPower(-frontLeftPower*0.6);
        backLeft.setPower(-backLeftPower*0.6);
        frontRight.setPower(-frontRightPower*0.6);
        backRight.setPower(-backRightPower*0.6);
    }
}
