package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

        if(gamepad1.y){
            bot.aimer.start();
        }

        if(gamepad1.x){
            bot.aimerPos = bot.odometry.getAll();
        }

        if(gamepad2.x){
//            optimizer.show();
            bot.wobbleGoal.start();
        }

        if(gamepad2.y){
            bot.shooter.start();
        }

//

//
//        if(gamepad2.right_trigger > 0){
//            bot.extendWobbleGoal(1);
//        }else if (gamepad2.left_trigger > 0 ){
//            bot.extendWobbleGoal(-1);
//        }else{
//            bot.extendWobbleGoal(0);
//        }
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

        bot.optimizeOdometry();
        
//        telemetry = telemetryHandler.addAutoAimer(telemetry, bot);
//        telemetry = telemetryHandler.addOuttake(telemetry, bot);

//        telemetry.addData("cll pos", bot.cll.getPosition());
//        telemetry.addData("clr pos", bot.clr.getPosition());
//        telemetryHandler.addAutoAimer();
////        telemetryHandler.addAngularPosition();
//        telemetry = telemetryHandler.getTelemetry();
////
//        telemetry = telemetryHandler.getTelemetry();


//        if(bot.shooter.inited){
//            telemetry.addData("executing", bot.shooter.autoModuleThread.executing);
//        }
//        telemetry.update();
//


//        telemetryHandler.addAngularPosition();
//        telemetryHandler.addOdometry();
//        telemetry.addData("avgdeltatime", optimizer.avgDeltaTime);
//        telemetry.addData("heading", bot.odometry.h);
//        telemetry.addData("heading", Optimizer.optimizeHeading(bot.odometry.h));
        telemetry.addData("wgStart", bot.wgStartMode);
        telemetry.update();
    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
        bot.stopOdoThread();
    }
}
