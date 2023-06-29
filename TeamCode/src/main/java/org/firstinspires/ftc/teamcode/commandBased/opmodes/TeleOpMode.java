package org.firstinspires.ftc.teamcode.commandBased.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.rr.util.DashboardUtil;

public class TeleOpMode extends BaseOpMode {

    protected DrivetrainSubsystem drivetrainSS;

    protected TelemetryPacket packet;
    protected Canvas fieldOverlay;

    @Override
    public void initialize() {
        drivetrainSS = new DrivetrainSubsystem(hardwareMap);
        packet = new TelemetryPacket();
        fieldOverlay = packet.fieldOverlay();
        super.initialize();

        tele.addLine("Ready to start:");
        tele.update();
    }

    @Override
    public void run() {
        super.run();

        if (Constants.DISPLAY) {
            // Draw the target on the field
            fieldOverlay.setStroke("#dd2c00");
            fieldOverlay.strokeCircle(Constants.TARGET.getX(), Constants.TARGET.getY(), 3);

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, drivetrainSS.getPose());

            //send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}
