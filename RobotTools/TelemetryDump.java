package org.firstinspires.ftc.teamcode.rework.RobotTools;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class TelemetryDump {
    Telemetry telemetry;
    private static List<TelemetryProvider> providers = new ArrayList<>();

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static void registerProvider(TelemetryProvider provider) {
        providers.add(provider);
    }
    public static void removeProvider(TelemetryProvider provider) {
        providers.remove(provider);
    }

    public void update() {
        StringBuilder out = new StringBuilder();
        for(TelemetryProvider provider : providers) {
            for(Map.Entry<String, String> entry : provider.getTelemetryData().entrySet()) {
                out.append(entry.getKey() + entry.getValue() + "\n");
            }
        }
        telemetry.addLine(out.toString());
        telemetry.update();
    }
}
