package developing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import auto.AutoHandler;
import global.TerraBot;
import globalfunctions.Sleep;
import globalfunctions.TelemetryHandler;
@Disabled
@Autonomous(name="AutoRP", group="Auto")
public class AutoRP extends LinearOpMode {

    TerraBot bot = new TerraBot();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        telemetry.addData("Ready:", "Yes?");
        telemetry.update();
        waitForStart();

        double restPow = 0.15;
        while (opModeIsActive()) {
            bot.move(0, 0,restPow);
        }
    }
}
