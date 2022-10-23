package org.firstinspires.ftc.teamcode.robots.taubot.util;

import java.util.Map;

public interface TelemetryProvider {
    Map<String, Object> getTelemetry(boolean debug);
    String getTelemetryName();
}
