package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TankDrive {
    public DcMotorEx motorL;
    public DcMotorEx motorR;

    public TankDrive(DcMotorEx motorL, DcMotorEx motorR) {
        this.motorL = motorL;
        this.motorR = motorR;
    }

    public void motorLPower(double power) {
        motorL.setPower(power);
    }
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

