package com.wilyworks.simulator.framework;

import com.acmerobotics.dashboard.CustomVariableConsumer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.TreeMap;

/**
 * Wily Works implementation of the FTC Dashboard.
 */
public class WilyDashboard {
    static public TreeMap<String, String> data = new TreeMap<>();
    static public ArrayList<String> log = new ArrayList<>();
    static public Canvas fieldOverlay;
    static FtcDashboard instance = new FtcDashboard();

    public static FtcDashboard getInstance() { return instance; }

    public Telemetry getTelemetry() {
        return null; // ###
    }

    // Append the telemetry to our accumulated list. Everything will get rendered later when
    // WilyCore.update() is called.
    public void sendTelemetryPacket(TelemetryPacket telemetryPacket) {
        // https://www.baeldung.com/java-merge-maps
        telemetryPacket.data.forEach((key, value) -> this.data.put(key, value));
        log = telemetryPacket.log;
        fieldOverlay = telemetryPacket.fieldOverlay();
    }

    public void withConfigRoot(CustomVariableConsumer function) {
    }
}
