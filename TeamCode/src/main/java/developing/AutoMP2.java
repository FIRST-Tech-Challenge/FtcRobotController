package developing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import autofunctions.Path2;
import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.Sleep;
import globalfunctions.TelemetryHandler;

//@Disabled
@Autonomous(name="AutoMP2", group="Auto")
public class AutoMP2 extends LinearOpMode {

    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    Path2 path = new Path2(Constants.AUTO_START[0],Constants.AUTO_START[1],Constants.AUTO_START[2]);

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        telemetryHandler.init(telemetry, bot);
        bot.startOdoThreadAuto(this, false);
        telemetry.addData("Ready:", "Yes?");
        telemetry.update();
        waitForStart();
        path.addSetpoint(30,30,0);
        path.start(bot, this);


        bot.stopOdoThread();
    }
}
