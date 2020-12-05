package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
import util.Vector;

@TeleOp(name = "TerraOp V5")
public class TerraOp extends OpMode {

    TerraBot bot = new TerraBot();

    double vs = 0;




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
        vs = bot.getVoltageScale();
        bot.outtakeStartL *= vs;
        bot.outtakeStartR *= vs;

        bot.setLEDs(0,255,0);


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
                    if(!bot.shooter.pausing) {
                        bot.defineShooter();
                    }
                    bot.shooter.start();
                    bot.outrController.setStartPow(bot.outtakeStartR);
                    bot.outlController.setStartPow(bot.outtakeStartL);
                }else{
                   // bot.definePowerShot();
                    bot.powerShot.start();
                    bot.outrController.setStartPow(bot.outtakeStartR*bot.powerShotSpeed);
                    bot.outlController.setStartPow(bot.outtakeStartL*bot.powerShotSpeed);
                }
            }
            if(gamepad2.x){
                bot.defineCalibrateCol();
                bot.calibrateCol.start();
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
                    bot.outtakeWithEncoders(1);
                }
            }
            
        }else if(bot.shooter.executing || bot.powerShot.executing){
            if(bot.powershot){
                bot.outtakeWithEncoders(bot.powerShotSpeed);
            }else {
                bot.outtakeWithEncoders(1);
            }
        }


        if(gamepad1.left_trigger> 0  && bot.timer.seconds() > 0.5){
            bot.fastmode = !bot.fastmode;
            bot.timer.reset();
        }

        if(gamepad1.right_trigger > 0 && bot.timer2.seconds() > 0.5){
            bot.powershot = !bot.powershot;
            bot.timer2.reset();
        }

        if (!bot.powershot && bot.gameTime.seconds() > 82 && bot.gameTime.seconds() < 90) {
            bot.powershot = true;
            bot.setLEDs(255,50,0);
        }

//        double forward = -gamepad1.right_stick_y;
//        double strafe = gamepad1.right_stick_x;
//        double turn = -gamepad1.left_stick_x;

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        if(bot.matchGyro) {
            if (!bot.powerShot.executing && !bot.goback.executing && !bot.calibrateCol.executing && !bot.shooter.executing) {
                if (Math.abs(strafe) < 0.2 || !bot.fastmode) {
                    bot.moveTeleOp(forward, strafe, turn);
                    bot.strafeMode = false;
                } else {
                    if (!bot.strafeMode) {
                        bot.savedHeading = bot.getHeading();
                        bot.strafeMode = true;
                    }
                    double err = (bot.getHeading() - bot.savedHeading);
                    turn = -Math.signum(err) * (0.25) + err * (-0.03);
                    bot.moveTeleOp(forward, strafe, turn);
                }
            }
        }else{
            if (!bot.powerShot.executing && !bot.goback.executing && !bot.calibrateCol.executing) {
                if (Math.abs(strafe) < 0.2 || !bot.fastmode) {
                    bot.moveTeleOp(forward, strafe, turn);
                    bot.strafeMode = false;
                } else {
                    if (!bot.strafeMode) {
                        bot.savedHeading = bot.getHeading();
                        bot.strafeMode = true;
                    }
                    double err = (bot.getHeading() - bot.savedHeading);
                    turn = -Math.signum(err) * (0.25) + err * (-0.03);
                    bot.moveTeleOp(forward, strafe, turn);
                }
            }
        }



//
//        if(!bot.globalMode) {
//            double forward = -gamepad1.right_stick_x;
//            double strafe = -gamepad1.right_stick_y;
//            double turn = -gamepad1.left_stick_x;
//
//            if (!bot.powerShot.executing && !bot.goback.executing && !bot.calibrateCol.executing) {
//                bot.moveTeleOp(forward, strafe, turn);
//            }
//        }else{
//            double forward = -gamepad1.right_stick_x;
//            double strafe = -gamepad1.right_stick_y;
//            double turn = -gamepad1.left_stick_x;
//
//            Vector vect = new Vector(strafe, forward);
//            Vector vect2 = vect.getRotatedVec(-bot.getHeading(), Vector.angle.DEGREES);
//
//
//            if (!bot.powerShot.executing && !bot.goback.executing && !bot.calibrateCol.executing) {
//                bot.moveTeleOp(vect2.y, vect2.x, turn);
//            }
//        }

//
//        telemetry.addData("Heading", bot.getHeading());
//        telemetry.addData("OdometryX", bot.odometry.getX());
//        telemetry.addData("OdometryY", bot.odometry.getY());
//        telemetry.update();
//        telemetry.addData("touch", bot.isTouchSensorPressed());
//        telemetry.update();
//
//        telemetry.addData("errR", bot.outrController.getPercentageError());
//        telemetry.addData("ce l", bot.outlController.currError);
//        telemetry.addData("ce r", bot.outrController.currError);
//        telemetry.addData("powl", bot.outlController.pow);
//        telemetry.addData("powr", bot.outrController.pow);
//        telemetry.addData("targetr", bot.outrController.targetSpeed);
//        telemetry.addData("targetl", bot.outlController.targetSpeed);
//        telemetry.addData("curr", bot.outrController.currSpeed);
//        telemetry.addData("curl", bot.outlController.currSpeed);
//        telemetry.addData("outr", bot.getOutrPos());
//        telemetry.addData("outl", bot.getOutlPos());
//        telemetry.addData("dsr1", bot.getDisR1());
//        telemetry.addData("dsl2", bot.getDisL2());
//        telemetry.update();

//        telemetry.addData("x", bot.odometry.getX());
//        telemetry.addData("y", bot.odometry.getY());
//        telemetry.addData("theta", bot.odometry.getTheta());
//        telemetry.addData("heading", bot.getHeading());

//        telemetry.addData("vs", vs);
//        telemetry.addData("dis", bot.getDisL2());

        telemetry.addData("gameTime", bot.gameTime.seconds());
        telemetry.addData("isPowerShotMode", bot.powershot);

        telemetry.update();


        bot.update();



    }

    @Override
    public void stop() {
       //bot.stopOdoThreadTele();
    }
}
