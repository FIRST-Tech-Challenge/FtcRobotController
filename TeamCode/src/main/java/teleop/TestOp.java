package teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;

@Disabled
@TeleOp(name = "TestOp V1")
public class TestOp extends OpMode {

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
    public void start() {
        bot.startOdoThreadTele();
    }

    @Override
    public void loop() {

        if(gamepad1.y){
            bot.calibrateCol.start();
        }

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        if(!bot.calibrateCol.executing) {
            bot.moveTeleOp(forward, strafe, turn);
        }

        telemetry.addData("heading", bot.getHeading());

        bot.update();

    }

    @Override
    public void stop() {
        bot.stopOdoThreadTele();
    }
}
