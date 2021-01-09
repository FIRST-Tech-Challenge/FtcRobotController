package org.firstinspires.ftc.teamcode.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class SLAM {
    T265Camera slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    final int robotRadius = 9; // inches
    TelemetryPacket packet = new TelemetryPacket();
    Canvas field = packet.fieldOverlay();
    T265Camera.CameraUpdate up;
    Translation2d translation;
    Rotation2d rotation;

    public void updateSLAMNav() {
        // We divide by 0.0254 to convert meters to inches
        up = slamra.getLastReceivedCameraUpdate();
        translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
    }
}
