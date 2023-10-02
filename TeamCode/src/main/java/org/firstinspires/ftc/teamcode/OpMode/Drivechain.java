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
}
