package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;

public abstract class AbstractSubsystem {
    public AbstractRobot robot;
    //public Gamepad gamepad1, gamepad2;
    public final Telemetry telemetry;
    public boolean usesConfig = false;
    public AbstractSubsystem(AbstractRobot robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;
    }

    public abstract void init() throws IOException;

    public abstract void start();

    public abstract void driverLoop(

    );

    public abstract void stop();
}
