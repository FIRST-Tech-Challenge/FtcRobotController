package org.firstinspires.ftc.teamcode.AbstractClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public abstract class AbstractRobot {
    public ArrayList<AbstractSubsystem> subsystems;
    public final OpMode opMode;
    public final Telemetry telemetry;

    public AbstractRobot(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        subsystems = new ArrayList<>();

    }
    public void driverLoop()
    {
        for(AbstractSubsystem  system : subsystems)
        {
            system.driverLoop();
        }
    }
}
