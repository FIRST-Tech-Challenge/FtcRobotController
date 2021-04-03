package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.TelemetryHandler;

@TeleOp(name = "TerraOp")
public class TerraOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    @Override
    public void init() {
        bot.init(hardwareMap);
        telemetry.addData("Ready", "");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.moveTeleOp(forward, strafe, turn);

        if(gamepad1.right_trigger > 0){
            bot.fastMode = true;
        }else if(gamepad1.left_trigger > 0){
            bot.fastMode = false;
        }


        if(!bot.areAutomodulesRunning()) {
            bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);

            bot.toggleOuttake(gamepad2.x);
            bot.outtakeWithCalculations();

            bot.pushRings(bot.pushControl.update(gamepad2.left_bumper, gamepad2.right_bumper));

        }

        if(gamepad2.y){
            bot.shooter.start();
        }

        if(gamepad2.right_trigger > 0){
            bot.extendWobbleGoal(1);
        }else if (gamepad2.left_trigger > 0 ){
            bot.extendWobbleGoal(-1);
        }else{
            bot.extendWobbleGoal(0);
        }

        if (gamepad2.dpad_right){
            bot.openClaw();
        } else if(gamepad2.dpad_left){
            bot.closeClaw();
        }


        bot.moveArm(-gamepad2.right_stick_y);

        telemetry.addData("Wobble goal pow", -gamepad2.right_stick_y);
        telemetry.addData("Wobble goal pos", -bot.arm.getCurrentPosition());
        telemetry.addData("Wobble goal pos in deg", bot.getArmPos());

//        bot.extendWobbleGoal(gamepad2.a);

//        bot.updateOdometry();

        
//        telemetry = telemetryHandler.addAutoAimer(telemetry, bot);
//        telemetry = telemetryHandler.addOuttake(telemetry, bot);
//        telemetry = telemetryHandler.addAngularPosition(telemetry, bot);
//
//        telemetry = telemetryHandler.addOdometry(telemetry, bot);

        telemetry.update();

    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
    }
}
