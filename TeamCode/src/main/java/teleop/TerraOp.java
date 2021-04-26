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

    boolean shouldICareAboutAuton = false;

    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.startOdoThreadTele();
//        bot.odometry.resetAll(Constants.TELE_START);
//        bot.angularPosition.resetGyro(0);
        optimizer.reset();

//        bot.readFromAuton();
        if(!shouldICareAboutAuton){
            bot.angularPosition.resetGyro(0);
        }
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);

    }



    @Override
    public void loop() {

        bot.initWobbleGoal();

        optimizer.update();

        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);

        bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);


        if(!bot.areAutomodulesRunning()) {
            bot.updateClaw(gamepad2.dpad_left, gamepad2.dpad_right);
            if(bot.wgStartMode > 2) {
                bot.moveArm(-gamepad2.right_stick_y);
            }

            bot.outtake(-gamepad2.right_stick_y);
            bot.shootRings(-gamepad2.left_stick_y);
        }

        if(gamepad1.x){
            bot.wobbleGoal.start();
        }

        if (gamepad1.y) {
            bot.shooter.start();
        }

        bot.outtakeWithCalculations();

        bot.optimizeOdometryHeading();
        bot.updateOdometryUsingSensors();
//        telemetry.addData("Powershot Mode", bot.powershotMode);
//        telemetry.addData("Can Move", bot.isMovementAvailable);
//        telemetry.addData("hasReached", bot.autoAimer.hasReached);
//        telemetry.addData("gyro", bot.angularPosition.getHeadingGY());
//        telemetry.addData("Dpos", Arrays.toString(bot.localizer.getPos()));
//        telemetry.addData("pos", Arrays.toString(bot.odometry.getPos()));
//        telemetry.addData("aim", Arrays.toString(bot.aimerPos));
        telemetryHandler.addAutoAimer();
//        telemetryHandler.addOdometry();
//        telemetryHandler.addAngularPosition();
//        telemetryHandler.addOuttake();
//        telemetryHandler.addOdometryRaw();
        telemetry = telemetryHandler.getTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
        bot.stopOdoThread();
    }
}
