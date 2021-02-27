package org.firstinspires.ftc.teamcode.toolkit.core;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public abstract class Subsystem {

    UpliftRobot robot;

    public Subsystem(UpliftRobot robot) {
        this.robot = robot;
    }

    public abstract void enable();
    public abstract void disable();
    public abstract void stop();
    public abstract void safeDisable();
}
