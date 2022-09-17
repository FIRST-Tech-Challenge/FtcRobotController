package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.*;

public class TankDriveDemo {

    public DcMotorEx motorL;
    public DcMotorEx motorR;

    public TankDriveDemo(DcMotorEx motorL, DcMotorEx motorR) {
        this.motorL = motorL;
        this.motorR = motorR;
    }

    // For stick control system
    public void motorLPower(double power) {
        motorL.setPower(power);
    }

    // For stick control system
    public void motorRPower(double power) {
        motorR.setPower(power);
    }

    public DcMotorEx getMotorL() {
        return motorL;
    }
    public DcMotorEx getMotorR() {
        return motorR;
    }
}
