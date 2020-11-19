package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;

@TeleOp(name = "TerraOp V5")
public class TerraOp extends OpMode {

    TerraBot bot = new TerraBot();




    @Override
    public void init() {

        telemetry.addData("Status: ","Not Ready");
        telemetry.update();
        bot.init(hardwareMap);

        telemetry.addData("Status: ","Ready");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        bot.resetArm();
        telemetry.addData("vel", bot.getArmVel());
        telemetry.update();
    }

    @Override
    public void start() {
        bot.turnArmWithEnc(50, 1);
        bot.turnWobbleArm(0.1);
        bot.turnControl.cur = 0.1;
        bot.gameTime.reset();
        //bot.startOdoThreadTele();
    }

    @Override
    public void loop() {

        if(!bot.autoModulesRunning()){
            if(!bot.autoModulesPaused()) {
                if(gamepad1.right_bumper){
                    bot.intaking = true;
                    bot.intake(bot.intakeSpeed);
                }else if(gamepad1.left_bumper){
                    bot.intaking = false;
                    bot.intake(-bot.intakeSpeed);
                }else if(bot.intaking){
                    bot.intake(bot.intakeSpeed);
                }else{
                    bot.intake(0);
                }
                
                if (gamepad2.right_stick_y < 0) {
                    bot.outtaking = true;
                    bot.outtake(0.41);
                } else if (gamepad2.right_stick_y > 0) {
                    bot.outtaking = false;
                    bot.outtake(-0.41);
                } else if (bot.outtaking) {
                    bot.outtake(0.41);
                } else {
                    bot.outtake(0);
                }

                double pr = bot.shootControlR.update(gamepad2.left_bumper, gamepad2.right_bumper);
                double pl = bot.shootControlL.update(gamepad2.left_bumper, gamepad2.right_bumper);

                bot.shoot(pr, pl);

                bot.lift(bot.liftControl.update(gamepad2.left_trigger, gamepad2.right_trigger));
                bot.turnWobbleArm(bot.turnControl.update(gamepad2.dpad_down, gamepad2.dpad_up, 0.5));
                bot.grab(bot.grabControl.update(gamepad2.dpad_left, gamepad2.dpad_right));

                if (bot.isArmInLimts(gamepad2.left_stick_y)) {
                    bot.turnArm(gamepad2.left_stick_y);
                } else {
                    bot.turnArm(0);
                }
                bot.resetShooterIs();
            }




            if(gamepad1.y){
                if(!bot.powershot) {
                    bot.shooter.start();
                }else{
                    bot.powerShot.start();
                }
            }
            if(gamepad2.x){
                bot.resetAll();
            }
            if(gamepad2.y){
                bot.defineGoback();
                bot.goback.start();
            }

            if(gamepad1.x){
                bot.wobbleGoal2.start();
            }

            if(bot.shooter.pausing || bot.powerShot.pausing){
                if(bot.powershot){
                    bot.outtakeWithEncoders(bot.powerShotSpeed);
                }else {
                    bot.outtakeWithEncoders(bot.outtakeSpeed);
                }
            }
            
        }else if(bot.shooter.executing || bot.powerShot.executing){
            if(bot.powershot){
                bot.outtakeWithEncoders(bot.powerShotSpeed);
            }else {
                bot.outtakeWithEncoders(bot.outtakeSpeed);
            }
        }


        if(gamepad2.left_stick_button && bot.timer.seconds() > 0.5){
            bot.fastmode = !bot.fastmode;
            bot.timer.reset();
        }

        if(gamepad1.right_trigger > 0 && bot.timer2.seconds() > 0.5){
            bot.powershot = !bot.powershot;
            bot.timer2.reset();
        }

//        if(!bot.powershot && bot.gameTime.seconds() > 85){
//            bot.powershot = true;
//        }

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        if(!bot.powerShot.executing && !bot.goback.executing) {
            bot.moveTeleOp(forward, strafe, turn);
        }

//
//        telemetry.addData("Heading", bot.getHeading());
//        telemetry.addData("OdometryX", bot.odometry.getX());
//        telemetry.addData("OdometryY", bot.odometry.getY());
//        telemetry.update();
//        telemetry.addData("touch", bot.isTouchSensorPressed());
//        telemetry.update();
//
//        telemetry.addData("errR", bot.outrController.getPercentageError());
       // telemetry.addData("gameTime", bot.gameTime.seconds());
        telemetry.addData("powl", bot.outlController.pow);
        telemetry.addData("powr", bot.outrController.pow);
        telemetry.addData("outr", bot.getOutrPos());
        telemetry.addData("outl", bot.getOutlPos());
        telemetry.update();


        bot.update();



    }

    @Override
    public void stop() {
        //bot.stopOdoThreadTele();
    }
}
