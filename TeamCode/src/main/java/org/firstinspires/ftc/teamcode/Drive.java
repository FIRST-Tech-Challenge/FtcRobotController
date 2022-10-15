package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    BNO055IMU imu;
    public boolean isFieldCentric;

    public Drive(DcMotor getFrontLeft,
                 DcMotor getBackLeft,
                 DcMotor getFrontRight,
                 DcMotor getBackRight,
                 BNO055IMU getImu) {
        motorFrontLeft = getFrontLeft;
        motorBackLeft = getBackLeft;
        motorFrontRight = getFrontRight;
        motorBackRight = getBackRight;
        imu = getImu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        isFieldCentric = false;
    }

    public void mecanum(double power, double strafe, double turn) {
        // denominator is largest motor power
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    public void fieldCentric(double power, double strafe, double turn) {
        double botHeading = -imu.getAngularOrientation().firstAngle;
        double rotationX = strafe * Math.cos(botHeading) - power * Math.sin(botHeading);
        double rotationY = strafe * Math.sin(botHeading) + power * Math.cos(botHeading);
        double denominator = Math.max(Math.abs(power) + Math.abs(strafe) + Math.abs(turn), 1);
        double frontLeftPower = (rotationY + rotationX + turn) / denominator;
        double backLeftPower = (rotationY - rotationX + turn) / denominator;
        double frontRightPower = (rotationY - rotationX - turn) / denominator;
        double backRightPower = (rotationY + rotationX - turn) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }
}
