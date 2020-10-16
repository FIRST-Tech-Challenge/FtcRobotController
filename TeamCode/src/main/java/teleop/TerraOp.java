package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;

@TeleOp(name = "TerraOp V1")
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
    public void loop() {
        if(gamepad1.right_bumper){
            bot.intaking = true;
            bot.intake(1);
        }else if(gamepad1.left_bumper){
            bot.intaking = false;
            bot.intake(-1);
        }else if(bot.intaking){
            bot.intake(1);
        }else{
            bot.intake(0);
        }

        if(gamepad2.right_bumper){
            bot.lift(0);
        }else if(gamepad2.left_bumper){
            bot.lift(1);
        }

        if(gamepad2.right_trigger > 0){
            bot.lift(1);
        }else if(gamepad2.left_trigger > 0){
            bot.lift(0);
        }

        if(gamepad2.dpad_up){
            bot.turnArm(0.4);
        }else if(gamepad2.dpad_down){
            bot.turnArm(0.8);
        }

        if (gamepad2.dpad_right){
            bot.grab(0.3);
        }else if(gamepad2.dpad_left){
            bot.grab(0.7);
        }

        bot.outtake(gamepad2.right_stick_y);
        bot.moveArm(gamepad2.left_stick_y);

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.move(forward, strafe, turn);


    }
}
