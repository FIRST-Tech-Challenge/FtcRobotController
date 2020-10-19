package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
import telefunctions.Cycle;
import telefunctions.ServoController;

@TeleOp(name = "TerraOp V1")
public class TerraOp extends OpMode {

    TerraBot bot = new TerraBot();

    ServoController turnControl = new ServoController(bot.turnStart, 0.0, 0.7);
//    ServoController grabControl = new ServoController(bot.grabStart, 0.45, 0.7);
//    ServoController liftControl = new ServoController(bot.liftStart, bot.liftStart, 0.9);
//    ServoController shootControl = new ServoController(bot.shootStart, bot.shootStart, 0.9);

    Cycle grabControl = new Cycle(bot.grabStart, 0.45);
    Cycle liftControl = new Cycle(bot.liftStart, 0.33, 1);
    Cycle shootControl = new Cycle(bot.shootStart, 0.2, 0.3);


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
            bot.intake(bot.intakeSpeed);
        }else if(gamepad1.left_bumper){
            bot.intaking = false;
            bot.intake(-bot.intakeSpeed);
        }else if(bot.intaking){
            bot.intake(bot.intakeSpeed);
        }else{
            bot.intake(0);
        }

        if(gamepad2.right_stick_y < 0){
            bot.outtaking = true;
            bot.outtake(bot.outtakeSpeed);
        }else if(gamepad2.right_stick_y > 0){
            bot.outtaking = false;
            bot.outtake(-bot.outtakeSpeed);
        }else if(bot.outtaking){
            bot.outtake(bot.outtakeSpeed);
        }else{
            bot.outtake(0);
        }


        //bot.lift(liftControl.update(gamepad2.right_trigger, gamepad2.left_trigger, 0.5));

        bot.lift(liftControl.update(gamepad2.right_trigger, gamepad2.left_trigger));

        bot.shoot(shootControl.update(gamepad2.left_bumper, gamepad2.right_bumper));



        //bot.shoot(shootControl.update(gamepad2.right_bumper, gamepad2.left_bumper, 0.4));

        bot.turnArm(turnControl.update(gamepad2.dpad_down, gamepad2.dpad_up, 0.5));

    //    bot.grab(grabControl.update(gamepad2.dpad_right, gamepad2.dpad_left, 0.5));

        bot.grab(grabControl.update(gamepad2.dpad_left, gamepad2.dpad_right));

        bot.moveArm(gamepad2.left_stick_y);

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.move(forward, strafe, turn);


        telemetry.addData("pos", bot.ssr.getPosition());
        telemetry.update();


    }
}
