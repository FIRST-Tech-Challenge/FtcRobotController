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


        telemetry.addData("average angle", bot.angularPosition.getHeading());
        telemetry.addData("gyro", bot.angularPosition.getHeadingGY());
        telemetry.addData("compass", bot.angularPosition.getHeadingCS());
//        telemetry.addData("left distance", bot.getLeftDistance());
//        telemetry.addData("front distance", bot.getFrontDistance());

//        telemetry.addData("distance to center: ", bot.autoAimer.getDisFromCenter(bot.getLeftDistance(), bot.angularPosition.getHeadingGY()));
//        telemetry.update();

//        telemetry.addData("Right Outtake Position", bot.outr.getCurrentPosition());
//        telemetry.addData("Left Outtake Position", bot.outl.getCurrentPosition());
//        telemetry.addData("Right Outtake Angular Velocity", bot.autoAimer.outrController.currSpeed);
//        telemetry.addData("Left Outtake Angular Velocity", bot.autoAimer.outlController.currSpeed);
//        telemetry.addData("Right Outtake Error", bot.autoAimer.outrController.currError);
//        telemetry.addData("Left Outtake Error", bot.autoAimer.outlController.currError);
//        telemetry.addData("Right Outtake Power", bot.autoAimer.outrController.power);
//        telemetry.addData("Left Outtake Power", bot.autoAimer.outlController.power);
//        telemetry.addData("Right Outtake Change Time", bot.autoAimer.outrController.changeTime);
//        telemetry.addData("Left Outtake Change Time", bot.autoAimer.outlController.changeTime);
//        telemetry.addData("Right Outtake Derivative Power", bot.autoAimer.outrController.derivativeOfPower);
//        telemetry.addData("Left Outtake Derivative Power", bot.autoAimer.outlController.derivativeOfPower);
//        telemetry.addData("Right Outtake Target Speed", bot.autoAimer.outrController.targetSpeed);
//        telemetry.addData("Left Outtake Target Speed", bot.autoAimer.outlController.targetSpeed);



//        telemetry.addData("Outl Target Power", bot.autoAimer.outlController.getMotorPower(bot.outl.getCurrentPosition()));

//        telemetry.addData("right wheel angular pos", bot.getRightAngPos());
//        telemetry.addData("target angular velocity", bot.rightSpeedController.targetSpeed);
        telemetry.update();





    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
    }
}
