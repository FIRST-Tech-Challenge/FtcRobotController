package org.firstinspires.ftc.teamcode.robots.refactorTrikeBot.subsystems;

import java.util.Map;

public interface Subsystem {
    Map<String, Object> getTelemetry();
    String getTelemetryName();
    void update();
    void stop();
}
