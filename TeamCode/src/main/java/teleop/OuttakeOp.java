package teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.TelemetryHandler;
//@Disabled
@TeleOp(name = "OuttakeOp")
public class OuttakeOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();


    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.startOdoThreadTele();
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);

        bot.angularPosition.resetGyro(0);
        bot.odometry.resetHeading(0);
        bot.updateOdoWithLocalizer();


    }

    @Override
    public void loop() {

        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);

        if (gamepad1.y) {
            if(!bot.powershotMode) {
                bot.shooter.start();
            }else{
                bot.powerShot.start();
            }
        }

        bot.outtakeWithCalculations();
        bot.optimizeOdometryHeading();
        bot.updateOdometryUsingSensors();
//        telemetryHandler.addTele(1,1,2,3,2);
//        telemetry = telemetryHandler.getTelemetry();

        telemetry.update();

    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
        bot.stopOdoThread();
    }
}
