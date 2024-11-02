package org.firstinspires.ftc.teamcode.Drivetrain.Utils;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class TelemetryTracking {
    TelemetryPacket packet;
    public TelemetryTracking() {
        packet = new TelemetryPacket();
        packet.fieldOverlay();

    }
    public TelemetryPacket updatePos (double xPos, double yPos,double thetaPos){
        packet.put("x", xPos);
        packet.put("y", yPos);
        packet.put("heading", thetaPos);
        return packet;
    }
}
