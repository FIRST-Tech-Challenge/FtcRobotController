package com.bravenatorsrobotics.drive;

import com.bravenatorsrobotics.core.Robot;

public abstract class AbstractDrive {

    protected final Robot robot;

    public AbstractDrive(Robot robot) {
        this.robot = robot;
    }

    public abstract void Drive(double v, double h, double r);

}
