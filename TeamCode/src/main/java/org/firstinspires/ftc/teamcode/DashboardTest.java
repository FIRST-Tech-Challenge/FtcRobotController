package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="Dashboard Test", group="Testing")
public class DashboardTest extends OpMode {
    public static int OP_MODE_TEST_VALUE = 10;

    private int loopCount;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        loopCount = 0;
        resetRuntime();
    }

    @Override
    public void init_loop() {
        loop();
    }

    @Override
    public void loop() {
        telemetry.addData("Loop count", ++loopCount);
        telemetry.addData("Elapsed seconds", "%.1f", getRuntime());
        telemetry.addData("TEST_VALUE", RobotConfig.TEST_VALUE);
        telemetry.addData("OP_MODE_TEST_VALUE", OP_MODE_TEST_VALUE);
        telemetry.update();
    }
}
