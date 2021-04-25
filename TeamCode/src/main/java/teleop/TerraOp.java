package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

import global.TerraBot;
import globalfunctions.Optimizer;
import globalfunctions.TelemetryHandler;

@TeleOp(name = "TerraOp")
public class TerraOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    Optimizer optimizer = new Optimizer();

    boolean shouldICareAboutAuton = true;

    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.startOdoThreadTele();
//        bot.odometry.resetAll(Constants.TELE_START);
//        bot.angularPosition.resetGyro(0);
        optimizer.reset();

        bot.readFromAuton();
        if(!shouldICareAboutAuton){
            bot.angularPosition.resetGyro(0);
        }
//        bot.updateOdoWithSensors();

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

        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger);

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

//            bot.updateRP(gamepad2.left_bumper, gamepad2.right_bumper);
            bot.updateClaw(gamepad2.dpad_left, gamepad2.dpad_right);
//            if(bot.wgStartMode > 2) {
//                bot.moveArm(-gamepad2.right_stick_y);
//            }

        }

        bot.outtake(gamepad2.right_stick_y);
        if (!bot.intaking) { bot.rs.setPower(Math.signum(gamepad2.left_stick_y/5    )); }
//

//        if(gamepad1.b){
////            bot.aimerPos = Constants.AUTO_SHOOT_POS;
//            bot.powerShot.start();
//        }

        if(gamepad1.x){
//            optimizer.show();
            bot.wobbleGoal.start();
        }

        if (gamepad1.y) {
//                if (bot.powershotMode) {
//                    bot.powerShot.start();
//                } else {
////                    bot.aimer.start();
//                    bot.shooter.start();
////                    if(bot.autoAimer.hasReached) {
////                        bot.shooter.start();
////                    }
//                }
            bot.shooter.start();
        }


        if(bot.powerShotController.isPressing(gamepad1.left_trigger>0)){
            bot.powershotMode = !bot.powershotMode;
            if(bot.powershotMode){
                bot.definePowershot();
            }
//            else{
//                bot.defineShooterAgain();
//            }
        }

        bot.optimizeOdometryHeading();
        if (gamepad2.x) {
            bot.updateOdoWithGyro();
            bot.updateLocalizerWithHeading();
            bot.updateOdoWithLocalizer();
            bot.aimerPos = bot.odometry.getAll();
        }
//        telemetry.addData("wgStart", bot.wgStartMode);
//        telemetryHandler.addOdometry();
//        telemetryHandler.addAutoAimer();
//        telemetry = telemetryHandler.getTelemetry();
        telemetry.addData("Powershot Mode", bot.powershotMode);
        telemetry.addData("Can Move", bot.isMovementAvailable);
        telemetry.addData("hasReached", bot.autoAimer.hasReached);
        telemetry.addData("gyro", bot.angularPosition.getHeadingGY());
        telemetry.addData("Dpos", Arrays.toString(bot.localizer.getPos()));
        telemetry.addData("pos", Arrays.toString(bot.odometry.getPos()));
        telemetry.addData("aim", Arrays.toString(bot.aimerPos));

//        telemetry.addData("Wg Start Pos", bot.wgStart);
//        telemetry.addData("Gyro", bot.angularPosition.getHeadingGY());
        telemetry.update();
    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
        bot.stopOdoThread();
    }
}
