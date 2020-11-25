package teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;

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
//
        if(gamepad1.y){
            bot.updateLocalizer();
        }

        if(gamepad1.x){
            bot.resetAll();
            bot.heading = bot.localizer.getAngle();
        }

        telemetry.addData("X", bot.localizer.getX());
        telemetry.addData("Y", bot.localizer.getY());
        telemetry.addData("Angle", bot.localizer.getAngle());

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.moveTeleOp(forward, strafe, turn);

//        telemetry.addData("x", bot.odometry.getX());
//        telemetry.addData("y", bot.odometry.getY());
//        telemetry.addData("theta", bot.odometry.getTheta());

        telemetry.update();


    }

    @Override
    public void stop() {
        bot.stopOdoThreadTele();
    }
}
