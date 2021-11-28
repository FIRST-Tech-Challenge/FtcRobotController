package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trajectory.Parametric;

/** This opmode displays a linear approximation of the spline from -50, 50 (facing right) to (50, 50) (facing right)
 *  It also calculates spline length
 * @author TheConverseEngineer
 */
@TeleOp(name="Parametric Test", group="Tests")
public class ParametricTest extends OpMode {

    Parametric parametric;
    FtcDashboard dashboard;

    @Override
    public void init() {
        parametric = new Parametric(-50d, -50d, 0d, 50d, 50d, 50d, 0d, 50d);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setStroke("goldenrod")
                .strokePolyline(parametric.approxXLine(7), parametric.approxYLine(7));
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("Spline Length", parametric.approxLength());
    }
}
