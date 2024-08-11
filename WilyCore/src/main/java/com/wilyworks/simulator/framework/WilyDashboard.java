package com.wilyworks.simulator.framework;

import com.acmerobotics.dashboard.CustomVariableConsumer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Func;
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
    private Telemetry telemetry = new TelemetryAdapter();

    public static FtcDashboard getInstance() { return instance; }

    public Telemetry getTelemetry() { return telemetry; }

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

    public void clearTelemetry() {

    }

    private static class LogAdapter implements Telemetry.Log {
        private TelemetryPacket telemetryPacket;

        private LogAdapter(TelemetryPacket packet) {
            telemetryPacket = packet;
        }

        @Override
        public int getCapacity() {
            return 0;
        }

        @Override
        public void setCapacity(int capacity) {

        }

        @Override
        public DisplayOrder getDisplayOrder() {
            return DisplayOrder.OLDEST_FIRST;
        }

        @Override
        public void setDisplayOrder(DisplayOrder displayOrder) {

        }

        @Override
        public void add(String entry) {
            telemetryPacket.addLine(entry);
        }

        @Override
        public void add(String format, Object... args) {
            telemetryPacket.addLine(String.format(format, args));
        }

        @Override
        public void clear() {
            telemetryPacket.clearLines();
        }
    }

    private class TelemetryAdapter implements Telemetry {
        private TelemetryPacket currentPacket;
        private LogAdapter log;

        public TelemetryAdapter() {
            currentPacket = new TelemetryPacket();
            log = new LogAdapter(currentPacket);
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return addData(caption, String.format(format, args));
        }

        @Override
        public Item addData(String caption, Object value) {
            currentPacket.put(caption, value);
            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            throw new UnsupportedOperationException();
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean removeItem(Item item) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void clear() {
            clearTelemetry();

            currentPacket = new TelemetryPacket();
            log = new LogAdapter(currentPacket);
        }

        @Override
        public void clearAll() {
            clear();
        }

        @Override
        public Object addAction(Runnable action) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean removeAction(Object token) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void speak(String text) {
            throw new UnsupportedOperationException();
        }

        @Override
        public void speak(String text, String languageCode, String countryCode) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean update() {
            sendTelemetryPacket(currentPacket);

            currentPacket = new TelemetryPacket();
            log = new LogAdapter(currentPacket);

            return true;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            currentPacket.addLine(lineCaption);
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            throw new UnsupportedOperationException();
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {
            throw new UnsupportedOperationException();
        }

        @Override
        public int getMsTransmissionInterval() {
            return 0;
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {
        }

        @Override
        public String getItemSeparator() {
            return null;
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return null;
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public void setDisplayFormat(DisplayFormat displayFormat) {

        }

        @Override
        public Log log() {
            return log;
        }
    }

}
