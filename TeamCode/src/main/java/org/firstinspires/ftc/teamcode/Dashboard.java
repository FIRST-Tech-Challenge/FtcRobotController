package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import java.util.ArrayList;


@Config
public class Dashboard extends BlocksOpModeCompanion {
    static FtcDashboard dashboard;

    public static ArrayList<Double> infiniteTelemetry = new ArrayList<>();



    @ExportToBlocks(
            parameterLabels = {}
    )
    public static void InitDashboard() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        infiniteTelemetry.clear();
        TelemetryPacket packet = new TelemetryPacket();
    }
    @ExportToBlocks(
            parameterLabels = {"key", "text"}
    )
    public static void Telemetry_with_Text(String key, String text) {
        telemetry.addData(key, text);
    }

    @ExportToBlocks(
            parameterLabels = {}
    )
    public static void TelemetryUpdate() {
        telemetry.addData("size",infiniteTelemetry.size());
        telemetry.update();
    }

    @ExportToBlocks(
            parameterLabels = {"key", "number"}
    )
    public static void Telemetry_with_number(String key, double number) {
        telemetry.addData(key, number);
    }


    @ExportToBlocks(
            parameterLabels = {"X", "Y","orientation"}
    )
    public static void Trajectory(double X, double Y,double orientation) {
        TelemetryPacket packetPos = new TelemetryPacket();

        packetPos.fieldOverlay()
                .drawImage("/robot.png", X/ 2.54+9, -Y / 2.54 + 135, 18, 18,Math.toRadians(orientation-90),9,9 ,true);
        infiniteTelemetry.add(Y);
        infiniteTelemetry.add(-X);
        for (int count = 0; count < infiniteTelemetry.size(); count += 2) {
            packetPos.fieldOverlay()
                    .setFill("blue")
                    .fillCircle(infiniteTelemetry.get(count) / 2.54 - 63, (infiniteTelemetry.get(count + 1)) / 2.54 + 63,0.5);
        }
        dashboard.sendTelemetryPacket(packetPos);
    }

}
