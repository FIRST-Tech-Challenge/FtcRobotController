package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import global.TerraBot;
import globalfunctions.Storage;
import globalfunctions.TelemetryHandler;

@TeleOp(name = "TerraOp")
public class TerraOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    Storage storage = new Storage();

    @Override
    public void init() {
        bot.init(hardwareMap);
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);
        bot.shooter.autoModuleThread.executing = false;
//        bot.moveArmWithEnc(45, 1);
    }

    @Override
    public void loop() {
        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger,gamepad1.left_trigger);
        bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);

        bot.updateRP(gamepad2.left_bumper, gamepad2.right_bumper);


        if(!bot.areAutomodulesRunning()) {


            bot.toggleOuttake(gamepad2.x);
            bot.outtakeWithCalculations();

            bot.pushRings(bot.pushControl.update(gamepad2.left_bumper, gamepad2.right_bumper));

            bot.claw(bot.cllControl.update(gamepad2.dpad_left, gamepad2.dpad_right), bot.clrControl.update(gamepad2.dpad_left, gamepad2.dpad_right));

        }

//        bot.outtake(gamepad2.right_stick_y);
//
        if(gamepad2.y){
            bot.shooter.start();
        }
//
//
        if(gamepad2.right_trigger > 0){
            bot.extendWobbleGoal(1);
        }else if (gamepad2.left_trigger > 0 ){
            bot.extendWobbleGoal(-1);
        }else{
            bot.extendWobbleGoal(0);
        }
//
//        if (gamepad2.dpad_right){
//            bot.openClaw();
//        } else if(gamepad2.dpad_left){
//            bot.closeClaw();
//        }
//
//
//        bot.moveArm(-gamepad2.right_stick_y);
//        telemetry.addData("WGE Target Pos", bot.updateWge());

//        telemetry.addData("Wobble goal pow", -gamepad2.right_stick_y);
//        telemetry.addData("Wobble goal pos", -bot.arm.getCurrentPosition());
//        telemetry.addData("Wobble goal pos in deg", bot.getArmPos());
//        telemetry.addData("WGE Pos", bot.getWgePos());

//        bot.extendWobbleGoal(gamepad2.a);

//        bot.updateOdometry();

        
//        telemetry = telemetryHandler.addAutoAimer(telemetry, bot);
//        telemetry = telemetryHandler.addOuttake(telemetry, bot);
//        telemetry = telemetryHandler.addAngularPosition(telemetry, bot);
//
//        telemetry = telemetryHandler.addOdometry(telemetry, bot);
//        telemetry.addData("cll pos", bot.cll.getPosition());
//        telemetry.addData("clr pos", bot.clr.getPosition());
//        telemetryHandler.addAutoAimer();
////        telemetryHandler.addAngularPosition();
//        telemetry = telemetryHandler.getTelemetry();
//
//        telemetry.update();

    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
//        storage.makeOutputFile("Today");
//        storage.saveText("yes", "pls work");
    }
}
