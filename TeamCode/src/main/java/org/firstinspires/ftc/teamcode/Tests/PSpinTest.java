package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.Input;

@Config
@TeleOp(name = "Proportional spin test")
public class PSpinTest extends LinearOpMode {

    public static int setPoint = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Input input = new Input(hardwareMap, true);
        input.resetIMU();

        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);

        waitForStart();

        while (opModeIsActive()) {
            input.spinToPosition(setPoint);
            BotTelemetry.update();
        }
    }
}
