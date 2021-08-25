package tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
import globalfunctions.TelemetryHandler;

@TeleOp(name = "TestOp")
public class TestOp extends OpMode {
    // Define the bot, telemetryHandler, optimizer
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    @Override
    public void init() {
        telemetry.addData("Ready?", "No.");
        telemetry.update();
        bot.teleInit(hardwareMap);
        telemetryHandler.init(telemetry, bot);
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();

    }

    @Override
    public void start(){
    }



    @Override
    public void loop() {
        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);

        if (bot.outtakeButtonController.isPressedOnce(gamepad1.y)) {
            bot.shooter.start();
        }
        bot.updateAutoModules();
    }

    @Override
    public void stop() {
        bot.stop();
    }
}
