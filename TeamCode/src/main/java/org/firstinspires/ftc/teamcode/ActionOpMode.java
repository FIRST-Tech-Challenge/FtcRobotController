package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class ActionOpMode extends LinearOpMode {
    private final FtcDashboard dash = FtcDashboard.getInstance();

    protected void runBlocking(Action a) {
        Canvas c = new Canvas();
        a.preview(c);

        boolean b = true;
        while (b && !isStopRequested()) {
            TelemetryPacket p = new TelemetryPacket();
            p.fieldOverlay().getOperations().addAll(c.getOperations());

            b = a.run(p);

            dash.sendTelemetryPacket(p);
        }
    }
}
