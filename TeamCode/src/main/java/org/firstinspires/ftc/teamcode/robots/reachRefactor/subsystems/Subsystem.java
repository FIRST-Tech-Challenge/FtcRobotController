package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.TelemetryProvider;

public interface Subsystem extends TelemetryProvider {
    void update();
    void reset();
    void stop();
}
