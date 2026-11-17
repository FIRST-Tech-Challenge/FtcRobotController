package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DcMotorWrapper {
    private final DcMotor dcMotor;
    private double ratio;

    public DcMotorWrapper(DcMotor dcMotor, double ratio) {
        this.dcMotor = dcMotor;
        this.ratio = ratio;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        dcMotor.setDirection(direction);
    }

    public void setPower(double power) {
        dcMotor.setPower(power);
    }

    public double getPower() {
        return dcMotor.getPower();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
