package developing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestOdometry")
public class TestOdometry extends OpMode {
    TestRobot bot = new TestRobot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    @Override
    public void init() {
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        bot.updateOdometry();
        telemetry = telemetryHandler.addOdometry(telemetry, bot);
        telemetry.update();

    }

    @Override
    public void stop() { }
}
