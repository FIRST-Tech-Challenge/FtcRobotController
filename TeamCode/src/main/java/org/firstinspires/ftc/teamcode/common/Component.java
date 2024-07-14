package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component {

    protected Telemetry telemetry;

    public Component(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public abstract void log();

    public abstract void update();

}
