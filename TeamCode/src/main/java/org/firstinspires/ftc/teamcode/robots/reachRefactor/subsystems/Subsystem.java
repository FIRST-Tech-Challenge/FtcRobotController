package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import java.util.Map;

public interface Subsystem {
    Map<String, Object> getTelemetry();
    String getTelemetryName();
    void update();
    void stop();
}
