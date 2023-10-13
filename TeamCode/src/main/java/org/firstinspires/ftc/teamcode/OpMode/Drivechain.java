package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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

    public static void motorDirectionTest(double leftStick, double rightStick) { //REMEMBER LEFT SIDE MOTORS ARE REVERSED IN TELEOP
        frontLeft.setPower(leftStick);
        frontRight.setPower(leftStick);
        backLeft.setPower(leftStick);
        backRight.setPower(leftStick);
    }

    public static void drive(double leftStick, double rightStick) {

        //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        double y = -leftStick; // Remember, Y stick value is reversed
        double x = rightStick * 1.1; // Counteract imperfect strafing
        double rx = rightStick;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(-frontLeftPower);
        backLeft.setPower(-backLeftPower);
        frontRight.setPower(-frontRightPower);
        backRight.setPower(-backRightPower);
    }

            /*if(leftStick != 0) {
                if(rightStick > 0) {
                    frontLeft.setPower((leftStick + rightStick) / 2);
                    backRight.setPower((leftStick + rightStick) / 2);
                } else if(rightStick < 0) {
                    frontRight.setPower((leftStick + rightStick) / 2);
                    backLeft.setPower((leftStick + rightStick) / 2);
                } else {
                    frontLeft.setPower(leftStick);
                    frontRight.setPower(leftStick);
                    backLeft.setPower(leftStick);
                    backRight.setPower(leftStick);
                }
            } else {
                frontLeft.setPower(-rightStick);
                frontRight.setPower(rightStick);
                backLeft.setPower(-rightStick);
                backRight.setPower(rightStick);
            }
    }*/
}
