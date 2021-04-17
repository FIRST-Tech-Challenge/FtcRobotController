package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Scanner;

import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.Optimizer;
import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;

@TeleOp(name = "TerraOp")
public class TerraOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    Storage storage = new Storage();
    Optimizer optimizer = new Optimizer();
    double wgPos;

    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.startOdoThreadTele();
        bot.odometry.resetAll(Constants.TELE_START);
        bot.angularPosition.resetGyro(0);
        optimizer.reset();

        telemetry.addData("Ready?", "Yes!");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);
        storage.makeOutputFile("save");
        wgPos = Double.parseDouble(storage.readText("wgPos"));
//        bot.moveArmWithEnc(45, 1);
//

    }



    @Override
    public void loop() {
        bot.initWobbleGoal();

        optimizer.update();

        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad2.right_trigger);

//        if(bot.aimer.inited) {
//            if (!(bot.aimer.isExecuting() && !bot.aimer.pausing)) {
//                bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad2.right_trigger);
//            }
//        }else{
//            bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad2.right_trigger);
//        }

        bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);


        if(!bot.areAutomodulesRunning()) {
//            bot.toggleOuttake(gamepad2.x);
//            bot.outtakeWithCalculations();

            bot.updateRP(gamepad2.left_bumper, gamepad2.right_bumper);
            bot.updateClaw(gamepad2.dpad_left, gamepad2.dpad_right);
//            if(bot.wgStartMode > 2) {
//                bot.moveArm(-gamepad2.right_stick_y);
//            }

        }

//        bot.outtake(gamepad2.right_stick_y);
//

        if(gamepad1.b){
            bot.aimer.start();
        }

        if(gamepad2.x){
//            optimizer.show();
            bot.wobbleGoal.start();
        }

        if (gamepad1.y) {
            if(bot.powershotMode){
                bot.powerShot.start();
            }else{
                bot.shooter.start();
            }
        }


        if(bot.powerShotController.isPressing(gamepad2.left_trigger>0)){
            bot.powershotMode = !bot.powershotMode;
            if(bot.powershotMode){
                bot.definePowershot();
            }
//            else{
//                bot.defineShooterAgain();
//            }
        }

        bot.optimizeOdometry();
        if (gamepad1.x) {
            bot.setHeading(0);
            bot.updateLocalizer();
            bot.updateOdoWithSensors();
            bot.aimerPos = bot.odometry.getAll();
        }
//        telemetry.addData("wgStart", bot.wgStartMode);
//        telemetryHandler.addOdometry();
//        telemetryHandler.addAutoAimer();
//        telemetry = telemetryHandler.getTelemetry();
        telemetry.addData("Powershot Mode", bot.powershotMode);
        telemetry.addData("Can Move", bot.isMovementAvailable);
        telemetry.addData("Wg Start Pos", wgPos);
        telemetry.update();
    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
        bot.stopOdoThread();
    }
}
