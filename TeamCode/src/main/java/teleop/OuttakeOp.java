package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;

@TeleOp(name = "OuttakeOp")
public class OuttakeOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    @Override
    public void init() {
        bot.init(hardwareMap);
        telemetry.addData("Ready", "");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.moveTeleOp(forward, strafe, turn);

        if(gamepad1.right_trigger > 0){
            bot.fastMode = true;
        }else if(gamepad1.left_trigger > 0){
            bot.fastMode = false;
        }


        telemetryHandler.addAutoAimer();
//        telemetryHandler.addAngularPosition();
        telemetry = telemetryHandler.getTelemetry();

        telemetry.update();

    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
    }
}
