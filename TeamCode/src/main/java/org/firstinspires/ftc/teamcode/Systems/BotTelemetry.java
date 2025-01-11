package org.firstinspires.ftc.teamcode.Systems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotTelemetry { // maybe figure this out later

    private static Telemetry dashboardTelemetry;
    private static Telemetry telemetry;
    private static FtcDashboard dashboard;


    public static void setTelemetry(Telemetry tel, Telemetry dbTel) {
        dashboardTelemetry = dbTel;
        telemetry = tel;
        dashboard = FtcDashboard.getInstance();
    }
    public static <T> void addData(String parameter, T data) {
        dashboardTelemetry.addData(parameter, data);
        telemetry.addData(parameter, data);

    }
    public static void drawPosition(double x, double y, double heading) {

        // Send position data to FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("X Position (in)", x);
        packet.put("Y Position (in)", y);
        packet.put("Heading (deg)", Math.toDegrees(heading));

        // Draw robot position on dashboard (convert inches to feet)
        packet.fieldOverlay().setStrokeWidth(2)
                .setStroke("blue")
                .fillCircle(x / 12.0, y / 12.0, 0.2); // Robot position in feet

        dashboard.sendTelemetryPacket(packet);
    }


    public static void update() {
        dashboardTelemetry.update();
        telemetry.update();
    }
}
