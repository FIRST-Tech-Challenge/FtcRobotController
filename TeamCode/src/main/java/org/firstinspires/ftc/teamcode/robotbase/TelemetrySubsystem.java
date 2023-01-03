package org.firstinspires.ftc.teamcode.robotbase;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.ArrayList;

import java.util.function.Supplier;

public class TelemetrySubsystem extends SubsystemBase {
    private Telemetry telemetry;
    private Telemetry dashboardTelemetry;

    List<Pair<String, Supplier>> monitored = new ArrayList<Pair<String, Supplier>>();

    public TelemetrySubsystem(Telemetry telemetry, Telemetry dashboardTelemetry) {
        this.telemetry = telemetry;
        this.dashboardTelemetry = dashboardTelemetry;
    }

    public void addMonitor(String name, Supplier data) {
        monitored.add(Pair.create(name, data));
    }

    public void periodic() {
        for (Pair<String, Supplier> name_value : monitored) {
            telemetry.addData(name_value.first, name_value.second.get());
            dashboardTelemetry.addData(name_value.first, name_value.second.get());
        }
    }
}