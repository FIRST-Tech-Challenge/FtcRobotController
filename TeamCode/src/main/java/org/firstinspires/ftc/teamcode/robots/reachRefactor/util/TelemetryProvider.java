package org.firstinspires.ftc.teamcode.robots.reachRefactor.util;

import java.util.Map;

public interface TelemetryProvider {
    Map<String, Object> getTelemetry(boolean debug);
    String getTelemetryName();
}
