package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public static class FTCDashboardPackets {
    public static final FtcDashboard DASHBOARD = FTCDashboard.getInstance();
    private static TelemetryPacket packet;

    public FTCDashboardPackets() {
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
     * @param key - The string to be put into the keys of a packet
     * @param value - The string to be put into the values of a packet
     */
    public void put(String key, String value) {
        DASHBOARD.put(key, value);
    }

    /**
     * Sends the current packet to the dashboard
     * @param reinitializePacket - A boolean of whether or not to reinitialize the packet after it is sent.
     */
    public void send(Boolean reintializePacket) {
        DASHBOARD.sendTelemetryPacket();
        if (reintializePacket)
            createNewTelePacket();
    }
}