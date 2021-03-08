package developing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TestOp")
public class TestTele extends OpMode {
    TestRobot bot = new TestRobot();

    @Override
    public void init() {
        bot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.moveTeleOp(forward, strafe, turn);

        if(!bot.areAutomodulesRunning()) {
            bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);

            bot.outtake(gamepad2.right_stick_y);

            bot.pushRings(bot.pushControl.update(gamepad2.left_bumper, gamepad2.right_bumper));


        }

        if(gamepad1.a){
            bot.testModule.start();
        }



    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
    }
}
