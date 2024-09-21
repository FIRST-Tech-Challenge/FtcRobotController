package org.firstinspires.ftc.teamcode.Controllers;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class TelemetryTracking {
    static TelemetryPacket packet = new TelemetryPacket();
    public static void updatePos (double xPos, double yPos,double thetaPos){
        packet.fieldOverlay()
                .setRotation(thetaPos)
                .setFill("blue")
                .fillRect(xPos, yPos, 40, 40);
    }
}
