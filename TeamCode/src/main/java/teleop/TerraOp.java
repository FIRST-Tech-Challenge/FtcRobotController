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

    // Should the robot use the data stored from the last auton run?
    boolean shouldICareAboutAuton = false;

    @Override
    public void init() {
        // Initialize the robot
        bot.init(hardwareMap);
        // Start the odometry thread
        bot.startOdoThreadTele();

        // Reset odometry and angular position
//        bot.odometry.resetAll(Constants.TELE_START);
//        bot.angularPosition.resetGyro(0);

        // Reset optimizer
        optimizer.reset();


        // Use readings from last auton to find wg, robot, and angular positions
//        bot.readFromAuton();

        // If we don't want to use the last auton's data, reset the gyro to heading 0
        if(!shouldICareAboutAuton){
            bot.angularPosition.resetGyro(0);
            bot.odometry.resetHeading(0);
            bot.updateOdoWithLocalizer();
        }



        // Initialize telemetryHandler and tell the driver that initialization is done
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);

    }



    @Override
    public void loop() {

        // initialize the wobble goal arm (with several stages)
//        bot.initWobbleGoal();

        // update optimizer
        optimizer.update();

        // move the robot using the joysticks
        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);

        // update the intake with the gamepad1 bumpers
        bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);


        // if automodules aren't running:
        if(!bot.areAutomodulesRunning()) {

            // update the wobble goal claw using gamepad2's dpad left and right
            bot.updateClaw(gamepad2.dpad_left, gamepad2.dpad_right);

            // if the wobble goal is past stage two of initialization, the driver can move the arm using gamepad2 right joystick
            if(bot.wgStartMode > 2) {
                bot.moveArm(-gamepad2.left_stick_y);
            }

            // update outtake wheels manually with gamepad2's right stick y
            bot.outtake(-gamepad2.right_stick_y);

            // update the outtake servo manually with gamepad2's left stick y
            if(gamepad2.right_bumper) {
                bot.shootRings(Constants.RS_POW);
            }else if(gamepad2.left_bumper){
                bot.shootRings(-Constants.RS_POW);
            }else{
                bot.shootRings(0);
            }
        }

        // when the driver presses gamepad1.x, start the wobble goal automodule
        if(gamepad1.x){
            bot.wobbleGoal.start();
        }

        // when the driver presses gamepad1.y, start the shooting automodule
        if (gamepad1.y) {
            bot.shooter.start();
        }

        // update the outtake motor speeds if the automodule is running
        bot.outtakeWithCalculations();

        // use optimizer to fix odometry heading
        bot.optimizeOdometryHeading();

        // use the distance sensors and gyro to update odometry every once in a while
        bot.updateOdometryUsingSensors();

        // TELEMETRY BLOCK:
        telemetry.addData("localizerChecksFailed", bot.localizer.checksFailed);
        telemetry.addData("gyroChecksFailed", bot.angularPosition.checksFailed);

        telemetryHandler.addTele(0,0,0,0,0);
        telemetry = telemetryHandler.getTelemetry();
        telemetry.update();
    }

    @Override
    public void stop() {
        // stop the automodules and odometry thread
        bot.stopAllAutomodules();
        bot.stopOdoThread();
    }
}
