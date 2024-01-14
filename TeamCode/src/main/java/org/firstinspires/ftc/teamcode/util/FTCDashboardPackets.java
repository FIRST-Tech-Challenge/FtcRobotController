package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.io.IOException;

public class FTCDashboardPackets {
    public static final FtcDashboard DASHBOARD = FtcDashboard.getInstance();
    /**
     * The key to be used when putting a value into a packet.
     * Should be set as the name of the class that is using this.
     */
    private final String CONTEXT;
    private TelemetryPacket packet;
    private final boolean USE_LOGGING;

    public enum LoggingLevels {
        INFO,
        DEBUG {
            @Override
            public String get_level() {
                return "DEBUG";
            }
        },
        ERROR {
            @Override
            public String get_level() {
                return "ERROR";
            }
        },
        WARN {
            @Override
            public String get_level() {
                return "WARN";
            }
        };

        public String get_level() {
            return "INFO";
        }
    }

    public FTCDashboardPackets() {
        CONTEXT = "ROOT";
        USE_LOGGING = true;
        createNewTelePacket();
    }

    public FTCDashboardPackets(String context) {
        CONTEXT = context.toUpperCase();
        USE_LOGGING = true;
        createNewTelePacket();
    }

    public FTCDashboardPackets(boolean _useLogging) {
        CONTEXT = "ROOT";
        USE_LOGGING = _useLogging;
        createNewTelePacket();
    }

    /**
     * Creates a new telemetry packet, 
     * or initializes one if it does
     * not exist.
     */
    public void createNewTelePacket() {
        packet = new TelemetryPacket();
        if (USE_LOGGING) {
            try {
                Logging.setup();
            } catch (IOException e) {
                error(e, true, false);
            }
        }
    }

    private String getLoggingLevel(LoggingLevels level) {
        return level.get_level();
    }

    private void log(String value) {
        if (USE_LOGGING)
            Logging.log("%s:%s\t%s", CONTEXT, LoggingLevels.INFO.get_level(), value);
    }

    private void log(String value, LoggingLevels level) {
        if (USE_LOGGING)
            Logging.log("%s:%s\t%s", CONTEXT, getLoggingLevel(level), value);
    }

    private void log(String key, String value) {
        if (USE_LOGGING)
            Logging.log("%s:%s\t%s : %s", CONTEXT, LoggingLevels.INFO.get_level(), key, value);
    }

    private void log(String key, String value, LoggingLevels level) {
        if (USE_LOGGING)
            Logging.log("%s:%s\t%s : %s", CONTEXT, getLoggingLevel(level), key, value);
    }

    /**
     * Takes in a key and a value, and puts them both into the current packet.
     * @param key The string to be put into the keys of a packet
     * @param value The string to be put into the values of a packet
     */
    public void put(String key, String value) {
        packet.put(key, value);
        log(key, value);
    }

    /**
     * Takes in a value and puts it into the current packet.
     * @param value The string to be put into the values of a packet
     */
    public void put(String value) {
        packet.put(CONTEXT, value);
        log(value);
    }

    /**
     * Takes in an exception and puts it into the current packet.
     * @param e The exception to be put into the packet
     */
    public void error(Exception e) {
        packet.put("Error", e.getMessage());
        log(e.getMessage(), LoggingLevels.ERROR);
    }

    /**
     * Takes in an exception and puts it into the current packet.
     * @param message The message to be put into the packet
     */
    public void error(String message) {
        packet.put("Error", message);
        log(message, LoggingLevels.ERROR);
    }

    public void error(String message, boolean sendPacket) {
        error(message);
        if (sendPacket)
            send(false);
    }

    /**
     * Takes in an exception and puts it into the current packet.
     * @param e The exception to be put into the packet
     * @param sendPacket A boolean of whether or not to send the packet after the exception is put in
     * @param reinitializePacket A boolean of whether or not to reinitialize the packet after it is sent
     */
    public void error(Exception e, boolean sendPacket, boolean reinitializePacket) {
        error(e);
        if (sendPacket)
            send(reinitializePacket);
    }

    /**
     * Takes in a message and puts it into the current packet.
     * @param message The message to be put into the packet
     */
    public void warn(String message) {
        packet.put(CONTEXT + " : WARN", message);
        log(message);
    }

    /**
     * Takes in a message and puts it into the current packet.
     * @param message The message to be put into the packet
     * @param sendPacket Whether or not to send the packet
     */
    public void warn(String message, boolean sendPacket) {
        warn(message);
        if (sendPacket)
            send(false);
    }

    /**
     * Takes in a message and puts it into the current packet.
     * @param message The message to be put into the packet
     */
    public void info(String message) {
        packet.put(CONTEXT + " : INFO", message);
        log(message);
    }

    /**
     * Takes in a message and puts it into the current packet.
     * @param message The message to be put into the packet
     * @param sendPacket A boolean of whether or not to send the packet after the message is put in
     */
    public void info(String message, boolean sendPacket) {
        info(message);
        if (sendPacket)
            send(false);
    }

    /**
     * Takes in a message and puts it into the current packet.
     * @param message The message to be put into the packet
     * @param sendPacket A boolean of whether or not to send the packet after the message is put in
     * @param reinitializePacket A boolean of whether or not to reinitialize the packet after it is sent
     */
    public void info(String message, boolean sendPacket, boolean reinitializePacket) {
        info(message);
        if (sendPacket)
            send(reinitializePacket);
    }

    /**
     * Takes in a packet and puts it into the current packet.
     * @param packet The packet to be put into the packet
     */
    public void info(CommonPackets packet) {
        info(getCommonPacket(packet));
    }

    /**
     * Takes in a packet and puts it into the current packet.
     * @param packet The common message to be put into the packet
     * @param sendPacket A boolean of whether or not to send the packet after the packet is put in
     * @param reinitializePacket A boolean of whether or not to reinitialize the packet after it is sent
     */
    public void info(CommonPackets packet, boolean sendPacket, boolean reinitializePacket) {
        info(packet);
        if (sendPacket)
            send(reinitializePacket);
    }

    /**
     * Takes in a message and puts it into the current packet.
     * @param message The message to be put into the packet
     */
    public void debug(String message) {
        packet.put(CONTEXT + " : DEBUG", message);
        log(message, LoggingLevels.DEBUG);
    }

    /**
     * Takes in a message and puts it into the current packet, sends it,
     * but does not reinitialize the packet.
     * @param message The message to be put into the packet
     * @param sendPacket A boolean of whether or not to send the packet after the message is put in
     */
    public void debug(String message, boolean sendPacket) {
        debug(message);
        if (sendPacket)
            send(false);
    }

    /**
     * Takes in a message and puts it into the current packet.
     * @param message The message to be put into the packet
     * @param sendPacket A boolean of whether or not to send the packet after the message is put in
     * @param reinitializePacket A boolean of whether or not to reinitialize the packet after it is sent
     */
    public void debug(String message, boolean sendPacket, boolean reinitializePacket) {
        debug(message);
        if (sendPacket)
            send(reinitializePacket);
    }

    /**
     * Takes in a packet and puts it into the current packet.
     * @param packet The packet to be put into the packet
     */
    public void debug(CommonPackets packet) {
        debug(getCommonPacket(packet));
    }

    /**
     * Takes in a packet and puts it into the current packet.
     * @param packet The common message to be put into the packet
     * @param sendPacket A boolean of whether or not to send the packet after the packet is put in
     * @param reinitializePacket A boolean of whether or not to reinitialize the packet after it is sent
     */
    public void debug(CommonPackets packet, boolean sendPacket, boolean reinitializePacket) {
        debug(packet);
        if (sendPacket)
            send(reinitializePacket);
    }

    /**
     * Takes in a packet and returns the value of the packet
     * @param packet The common packet to retrieve the common message from
     * @return The value of the packet
     */
    public String getCommonPacket(CommonPackets packet) {
        return packet.getPacketMessage();
    }

    /**
     * Sends the current packet to the dashboard
     * @param reinitializePacket A boolean of whether or not to reinitialize the packet after it is sent.
     */
    public void send(boolean reinitializePacket) {
        DASHBOARD.sendTelemetryPacket(packet);
        if (reinitializePacket)
            createNewTelePacket();
    }

    public enum CommonPackets {
        START {
            @Override
            public String getPacketMessage() {
                return "Starting...";
            }
        },
        CONTINUE {
            @Override
            public String getPacketMessage() {
                return "Continuing...";
            }
        },
        WAITING {
            @Override
            public String getPacketMessage() {
                return "Waiting...";
            }
        };

        public String getPacketMessage() {
            return "Unknown Packet";
        }
    }
}