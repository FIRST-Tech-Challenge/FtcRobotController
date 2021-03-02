package developing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        bot.move(forward,strafe,turn);

        bot.updateIntake(gamepad1.left_bumper, gamepad1.right_bumper);

        bot.setCompassMode();

        bot.outtakeWithCalculations();
//        bot.outtake(gamepad2.right_stick_y);

        bot.pushRings(bot.pushControl.update(gamepad2.left_bumper, gamepad2.right_bumper));

    }
}
