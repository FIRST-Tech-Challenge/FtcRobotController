package org.firstinspires.ftc.teamcode.robots.LilVirani.util;

import java.util.Map;

public interface TelemetryProvider {
    Map<String, Object> getTelemetry(boolean debug);
    String getTelemetryName();
}
