package com.acmerobotics.dashboard.telemetry;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.ArrayList;
import java.util.Map;
import java.util.TreeMap;

/**
 * Thunk for FTC Dashboard's TelemetryPacket class.
 */
public class TelemetryPacket {
    public TreeMap<String, String> data;
    public ArrayList<String> log;
    public Canvas fieldOverlay;

    public TelemetryPacket() { this(true); }
    public TelemetryPacket(boolean drawDefaultField) {
        data = new TreeMap<>();
        log = new ArrayList<>();
        fieldOverlay = new Canvas();
    }
    public void put(String key, Object value) {
        data.put(key, value == null ? "null" : value.toString());
    }
    public void putAll(Map<String, Object> map) {
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            put(entry.getKey(), entry.getValue());
        }
    }
    public void addLine(String line) {
        log.add(line);
    }
    public void clearLines() {
        log.clear();
    }
    public Canvas fieldOverlay() {
        return fieldOverlay;
    }
}
