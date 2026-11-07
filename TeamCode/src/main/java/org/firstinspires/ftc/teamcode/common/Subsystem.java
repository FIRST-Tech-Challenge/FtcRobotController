package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//periodic runs every loop from Robot, and returns quickly. writeTelemetry gives values and information.
public abstract class Subsystem {
    public abstract void periodic();

    public abstract void writeTelemetry(Telemetry telemetry);
}
