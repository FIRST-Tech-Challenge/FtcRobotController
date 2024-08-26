package com.acmerobotics.dashboard.telemetry;

import com.acmerobotics.dashboard.canvas.Canvas;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

/**
 * Container for telemetry information. This class can be extended to support additional, custom
 * telemetry data.
 */
public class TelemetryPacket {
    private long timestamp;
    private SortedMap<String, String> data;
    private List<String> log;
    private Canvas field;
    private Canvas fieldOverlay;

    private static final Canvas DEFAULT_FIELD = new Canvas();

    static {
        DEFAULT_FIELD.setAlpha(0.4);
        DEFAULT_FIELD.drawImage("/dash/centerstage.webp", 0, 0, 144, 144);
        DEFAULT_FIELD.setAlpha(1.0);
        DEFAULT_FIELD.drawGrid(0, 0, 144, 144, 7, 7);
    }

    /**
     * Creates a new telemetry packet.
     */
    public TelemetryPacket(boolean drawDefaultField) {
        data = new TreeMap<>();
        log = new ArrayList<>();
        field = new Canvas();
        fieldOverlay = new Canvas();

        if (drawDefaultField) {
            field.getOperations().addAll(DEFAULT_FIELD.getOperations());
        }
    }

    public TelemetryPacket() {
        this(true);
    }

    /**
     * Stores a single key-value pair.
     *
     * @param key
     * @param value
     */
    public void put(String key, Object value) {
        data.put(key, value == null ? "null" : value.toString());
    }

    /**
     * Stores all entries of the provided map.
     *
     * @param map
     */
    public void putAll(Map<String, Object> map) {
        for (Map.Entry<String, Object> entry : map.entrySet()) {
            put(entry.getKey(), entry.getValue());
        }
    }

    /**
     * Adds a line to the telemetry log.
     *
     * @param line
     */
    public void addLine(String line) {
        log.add(line);
    }

    /**
     * Clears the telemetry log.
     */
    public void clearLines() {
        log.clear();
    }

    /**
     * Adds and returns the current timestamp to the packet. This is called automatically when the
     * packet is sent (and any previous timestamp will be overwritten).
     */
    public long addTimestamp() {
        timestamp = System.currentTimeMillis();
        return timestamp;
    }

    /**
     * Returns the field overlay canvas.
     */
    public Canvas fieldOverlay() {
        return fieldOverlay;
    }

    public Canvas field() {
        return field;
    }
}
