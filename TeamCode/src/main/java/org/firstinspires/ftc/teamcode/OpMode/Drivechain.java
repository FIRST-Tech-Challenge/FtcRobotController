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

    public static void drive(double leftStick, double rightStick) {

            if(leftStick != 0) {
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
                frontRight.setPower(-rightStick);
                backLeft.setPower(rightStick);
                backRight.setPower(rightStick);
            }
    }
}
