package org.firstinspires.ftc.teamcode.Systems;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotTelemetry { // maybe figure this out later

    FtcDashboard dashboard;
    Telemetry telemetry;

    public  BotTelemetry() {

        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

    }

    public void inputs() {
        //telemetry.addData("Move Input", gamepad1);
    }
}
