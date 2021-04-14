package teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import global.TerraBot;
import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;
@Disabled
@TeleOp(name = "OuttakeOp")
public class OuttakeOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();


    @Override
    public void init() {
        bot.init(hardwareMap);
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);
    }

    @Override
    public void loop() {

        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger);

        if(gamepad1.right_trigger > 0){
            bot.fastMode = true;
        }else if(gamepad1.left_trigger > 0){
            bot.fastMode = false;
        }


//        telemetryHandler.addAutoAimer();
//        telemetryHandler.addAngularPosition();
//        telemetry = telemetryHandler.getTelemetry();
//
//        telemetry.update();

    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
    }
}
