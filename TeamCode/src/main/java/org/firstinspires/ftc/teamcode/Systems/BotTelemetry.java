package org.firstinspires.ftc.teamcode.Systems;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BotTelemetry { // maybe figure this out later

    private static Telemetry dashboardTelemetry;
    private static Telemetry telemetry;

    public static <T> void addData(String parameter, T data) {
        dashboardTelemetry.addData(parameter, data);
        telemetry.addData(parameter, data);

    }

    public static void setTelemetry(Telemetry tel, Telemetry dbTel) {
        dashboardTelemetry = dbTel;
        telemetry = tel;
    }

    public static void update() {
        dashboardTelemetry.update();
        telemetry.update();
    }
}
