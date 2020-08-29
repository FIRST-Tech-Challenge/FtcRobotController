package org.firstinspires.ftc.teamcode.rework.RobotTools;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class TelemetryDump {
    Telemetry telemetry;
    private static ArrayList<TelemetryProvider> providers;

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.providers = new ArrayList<>();
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
            for(String entry : provider.getTelemetryData()) {
                out.append(entry + "\n");
            }
        }
        telemetry.addLine(out.toString());
        telemetry.update();
    }
}
