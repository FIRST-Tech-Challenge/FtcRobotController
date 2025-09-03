package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TankDrive {
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    public TankDrive(DcMotor left, DcMotor right) {
        this.leftMotor = left;
        this.rightMotor = right;

        //wasn't able to get telemetry working outside of an OpMode, but it must be possible
        //telemetry.addData("status", "Tank Drive Initialized");
    }

    public void drive(double leftPower, double rightPower, double speedFactor) {
        leftMotor.setPower(leftPower * speedFactor);
        rightMotor.setPower(rightPower * speedFactor);
    }
}
