package org.firstinspires.ftc.teamcode.robots.ri2d2023.util;

import java.util.Map;

public interface TelemetryProvider {
    Map<String, Object> getTelemetry(boolean debug);
    String getTelemetryName();
}
