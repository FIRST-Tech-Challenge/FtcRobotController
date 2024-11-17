package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryManager {
    private Telemetry telemetry;

    public TelemetryManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void update(String key, String value) {
        telemetry.addData(key, value);
        telemetry.update();
    }

    public void update(String key, double value) {
        telemetry.addData(key, value);
        telemetry.update();
    }

    public void clear() {
        telemetry.clear();
    }
}
