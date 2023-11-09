package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class FTCDashboardPackets {
    public static final FtcDashboard DASHBOARD = FtcDashboard.getInstance();
    /**
     * The key to be used when putting a value into a packet.
     * Should be set as the name of the class that is using this.
     */
    public static String CONTEXT = null;
    private static TelemetryPacket packet;

    public FTCDashboardPackets() {
        CONTEXT = "ROOT";
        createNewTelePacket();
    }

    public FTCDashboardPackets(String context) {
        CONTEXT = context.toUpperCase();
        createNewTelePacket();
    }

    /**
     * Creates a new telemetry packet, 
     * or initializes one if it does
     * not exist.
     */
    public void createNewTelePacket() {
        packet = new TelemetryPacket();
    }

    /**
     * Takes in a key and a value, and puts them both into the current packet.
     * @param key The string to be put into the keys of a packet
     * @param value The string to be put into the values of a packet
     */
    public void put(String key, String value) {
        packet.put(key, value);
    }

    /**
     * Takes in a value and puts it into the current packet.
     * @param value The string to be put into the values of a packet
     */
    public void put(String value) {
        packet.put(CONTEXT, value);
    }

    /**
     * Takes in an exception and puts it into the current packet.
     * @param e The exception to be put into the packet
     */
    public void error(Exception e) {
        packet.put("Error", e.getMessage());
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
    public void info(String message) {
        packet.put(CONTEXT + " : INFO", message);
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
        switch (packet) {
            case START:
                return "Starting...";
            case CONTINUE:
                return "Continuing...";
            case WAITING:
                return "Waiting...";
            default:
                return "Unknown packet";
        }
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
        START,
        CONTINUE,
        WAITING
    }
}