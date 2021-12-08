package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import java.util.Map;

public interface Subsystem {
    Map<String, Object> getTelemetry(boolean debug);
    String getTelemetryName();
    void update();
    void stop();
}
