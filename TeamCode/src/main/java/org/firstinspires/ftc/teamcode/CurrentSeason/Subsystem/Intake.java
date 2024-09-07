package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.PeppyFeetFiend;

public class Intake extends AbstractSubsystem {
    PeppyFeetFiend robot;
    public Intake(AbstractRobot robot) {
        super(robot);
        this.robot = (PeppyFeetFiend) robot;

    }

    @Override
    public void init() {
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        //use for telemetry
    }

    @Override
    public void stop() {

    }
}