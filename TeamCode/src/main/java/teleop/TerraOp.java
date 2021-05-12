package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.Optimizer;
import globalfunctions.TelemetryHandler;

@TeleOp(name = "TerraOp")
public class TerraOp extends OpMode {
    // Define the bot, telemetryHandler, optimizer
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    Optimizer optimizer = new Optimizer();

    @Override
    public void init() {
        telemetry.addData("Ready?", "No.");
        telemetry.update();
        bot.teleInit(hardwareMap);
        // Reset optimizer
        optimizer.reset();
        // Initialize telemetryHandler and tell the driver that initialization is done
        telemetryHandler.init(telemetry, bot);
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();

    }

    @Override
    public void start(){
        bot.gameTime.reset();
        bot.toggleKnockdown(true);
    }



    @Override
    public void loop() {
//        bot.fastMode = true;

        // initialize the wobble goal arm (with several stages)
//        if(shouldICareAboutAuton) {
//            bot.initWobbleGoal();
//        }
//        bot.initWobbleGoal();

        // update optimizer
        optimizer.update();

        // move the robot using the joysticks
        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);

        // update the intake with the gamepad1 bumpers
//        bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);

        // if automodules aren't running:
        if(!bot.areAutomodulesRunning()) {

            // update the wobble goal claw using gamepad2's dpad left and right
            bot.updateClaw(gamepad2.dpad_left, gamepad2.dpad_right);

            // if the wobble goal is past stage two of initialization, the driver can move the arm using gamepad2 right joystick
            if(bot.wgStartMode > 2) {
                bot.moveArm(-gamepad2.left_stick_y);
            }

            // update outtake wheels manually with gamepad2's right stick y
            if(bot.isOuttakeAvailable) {
                bot.outtake(-gamepad2.right_stick_y);
            }

            // update the outtake servo manually with gamepad2's left stick y
            if(bot.isOuttakeAvailable) {
                if (gamepad2.right_bumper) {
                    bot.shootRings(Constants.RS_POW);
                } else if (gamepad2.left_bumper) {
                    bot.shootRings(-Constants.RS_POW);
                } else {
                    bot.shootRings(0);
                }
            }
        }

        if(gamepad1.x){
            bot.powerShot.start();
        }

        // when the driver presses gamepad1.y, start the shooting automodule
        if (bot.outtakeButtonController.isPressedOnce(gamepad1.y)) {
            bot.shooter.start();
        }

        bot.toggleKnockdown(gamepad2.right_trigger > 0);

        // update the outtake motor speeds if the automodule is running
        bot.outtakeWithCalculations(true);

        // use optimizer to fix odometry heading within range [-180,180]
        bot.optimizeOdometryHeading();

        bot.updateAutoModules();
        // TELEMETRY
        telemetry.addData("pauses", Arrays.toString(bot.shooter.pauses.toArray()));
        telemetry.addData("pausing", bot.shooter.pausing);
        telemetry.addData("startstagenum", bot.shooter.startStageNum);
        telemetry.addData("currentstagenum", bot.shooter.stageNum);
        telemetry.addData("definestafenum", bot.shooter.defineStageNum);
//        telemetryHandler.addTele(0,0,0,0,0);
        telemetry.update();
    }

    @Override
    public void stop() {
        // stop the automodules and odometry thread
        bot.stop();
    }
}
