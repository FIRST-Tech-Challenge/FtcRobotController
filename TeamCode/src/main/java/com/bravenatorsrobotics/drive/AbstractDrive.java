package com.bravenatorsrobotics.drive;

import com.bravenatorsrobotics.core.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class AbstractDrive {

    protected final Robot robot;

    public AbstractDrive(Robot robot) {
        this.robot = robot;
    }

    // Abstract Methods
    protected abstract void SetAllPower(double power);
    protected abstract void SetPower(DcMotorEx motor, double power);

    public abstract void Drive(double v, double h, double r);
    public abstract void Stop();
}
