package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.spartronics4915.lib.T265Camera;

import java.util.List;

@Disabled
@Autonomous(name="Test T265", group="Iterative Opmode")
public class PositionTracking extends LinearOpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        slamra.start();

        slamra.stop();

    }
    final int robotRadius = 9; // inches
    TelemetryPacket packet = new TelemetryPacket();
    Canvas field = packet.fieldOverlay();
    T265Camera.CameraUpdate up = null;
    Translation2d translation = null;
    Rotation2d rotation = null;
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

    public double getRobotPosition(String xy) {
        updateSLAMNav();
        if (xy.equalsIgnoreCase("x")) {
            return translation.getX() / 0.0254;
        }
        else {
            if (xy.equalsIgnoreCase("y")) {
                return translation.getY() / 0.0254;
            }
        }
            return 0;
    }

}