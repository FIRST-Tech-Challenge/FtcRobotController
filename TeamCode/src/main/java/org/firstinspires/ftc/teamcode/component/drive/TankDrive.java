package org.firstinspires.ftc.teamcode.component.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TankDrive {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public TankDrive(DcMotor left, DcMotor right) {
        this.leftMotor = left;
        this.rightMotor = right;

        //wasn't able to get telemetry working outside of an OpMode, but it must be possible
        //telemetry.addData("status", "Tank Drive Initialized");
    }

    public void drive(double leftPower, double rightPower, double scaleFactor) {
        leftMotor.setPower(leftPower * scaleFactor);
        rightMotor.setPower(rightPower * scaleFactor);
    }
}
