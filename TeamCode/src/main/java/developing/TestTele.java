package developing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;

import global.Constants;

@TeleOp(name = "TestOp")
public class TestTele extends OpMode {
    TestRobot bot = new TestRobot();

    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.rightSpeedController.setTargetSpeed(Constants.MAX_OUTTAKE_SPEED/2);
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

            bot.outtake(-gamepad2.right_stick_y);

            bot.pushRings(bot.pushControl.update(gamepad2.left_bumper, gamepad2.right_bumper));

        }

        if(gamepad2.y){
            bot.shooter.start();
        }

//
//        telemetry.addData("gyro", bot.angularPosition.getHeadingGY());
//        telemetry.addData("compass", bot.angularPosition.getHeadingCS());
//        telemetry.addData("left distance", bot.getLeftDistance());
//        telemetry.addData("front distance", bot.getFrontDistance());

//        telemetry.addData("distance to center: ", bot.autoAimer.getDisFromCenter(bot.getLeftDistance(), bot.angularPosition.getHeadingGY()));
//        telemetry.update();

        telemetry.addData("right wheel angular pos", bot.getRightAngPos());
        telemetry.addData("target angular velocity", bot.rightSpeedController.targetSpeed);
        telemetry.update();





    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
    }
}
