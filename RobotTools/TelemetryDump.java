package org.firstinspires.ftc.teamcode.rework.RobotTools;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class TelemetryDump {
    Telemetry telemetry;
    HashMap<String, String> data;

    public TelemetryDump(Telemetry telemetry) {
        this.telemetry = telemetry;
        data = new HashMap<>();
    }

    public void addData(String s, String val) {
        data.put(s, val);
    }

    public void addData(String s, double val) {
        data.put(s, Double.toString(val));
    }

    public void update() {
        StringBuilder out = new StringBuilder();
        for (String key : data.keySet()) {
            out.append(key).append(data.get(key)).append("\n");
        }
        telemetry.addLine(out.toString());
        telemetry.update();
    }
}
