package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class FTCDashboardPackets {
    public static final FtcDashboard DASHBOARD = FtcDashboard.getInstance();
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
     * @param key The string to be put into the keys of a packet
     * @param value The string to be put into the values of a packet
     */
    public void put(String key, String value) {
        packet.put(key, value);
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
}