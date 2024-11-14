package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Elevator {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public Elevator(DcMotor LeftMotor, DcMotor RightMotor) {
        this.leftMotor = LeftMotor;
        this.rightMotor = RightMotor;
    }

    public void ChangePower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}
