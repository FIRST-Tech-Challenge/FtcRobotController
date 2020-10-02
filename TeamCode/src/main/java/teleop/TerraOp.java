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
        if(gamepad1.right_bumper) {
            bot.intake(-1);
        }else if(gamepad1.left_bumper){
            bot.intake(1);
        }else{
            bot.intake(0);
        }

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.move(forward, strafe, turn);

    }
}
