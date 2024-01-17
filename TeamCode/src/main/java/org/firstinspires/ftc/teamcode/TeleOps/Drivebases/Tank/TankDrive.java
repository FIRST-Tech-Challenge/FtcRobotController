package org.firstinspires.ftc.teamcode.TeleOps.Drivebases.Tank;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TankDrive {
    private DcMotor[] motors = new DcMotor[4];

    public TankDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        motors[0] = frontLeft;
        motors[1] = frontRight;
        motors[2] = backLeft;
        motors[3] = backRight;
    }

    public void drive(double leftPower, double rightPower) {
        if (Math.abs(leftPower) > 0.1) {
            motors[0].setPower(leftPower);
            motors[2].setPower(leftPower);
        } else {
            motors[0].setPower(0);
            motors[2].setPower(0);
        }
        if (Math.abs(rightPower) > 0.1) {
            motors[1].setPower(rightPower);
            motors[3].setPower(rightPower);
        } else {
            motors[1].setPower(0);
            motors[3].setPower(0);
        }
    }
}
