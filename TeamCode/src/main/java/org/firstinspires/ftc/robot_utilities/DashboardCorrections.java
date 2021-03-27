package org.firstinspires.ftc.robot_utilities;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;

public class DashboardCorrections {

    public static void drawRobotOnField(Pose2d pose, TelemetryPacket packet) {
        drawRobotOnField(pose, 13, "blue", packet);
    }

    public static void drawRobotOnField(Pose2d pose, double size, String color, TelemetryPacket packet) {
        double x = pose.getY();
        double y = -pose.getX();
        double heading_radians = pose.getHeading() + (Math.PI / 2);
        size = size/2;

        double[] xPoints = {x - size, x - size, x + size, x + size};
        double[] yPoints = {y - size, y + size, y - size, y + size};

        rotatePoints(xPoints, yPoints, heading_radians);

        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setStroke("goldenrod")
                .setFill(color)
                .fillPolygon(xPoints, yPoints);
    }

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }
}
