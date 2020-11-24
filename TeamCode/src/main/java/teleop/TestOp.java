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
            //bot.heading = bot.localizer.getAngle();
            bot.resetAll();
        }
//
        telemetry.addData("X", bot.localizer.getX());
        telemetry.addData("Y", bot.localizer.getY());
        telemetry.addData("Angle", bot.localizer.getAngle());
        telemetry.addData("x", bot.odometry.getX());
        telemetry.addData("y", bot.odometry.getY());
        telemetry.addData("theta", bot.odometry.getTheta());

//        telemetry.addData("Radius", bot.localizer.robotRadius);
//        telemetry.addData("dx", bot.localizer.dx);
//        telemetry.addData("dy", bot.localizer.dy);
//        telemetry.addData("r1", bot.localizer.r1);
//        telemetry.addData("l1", bot.localizer.l1);
//        telemetry.addData("r2", bot.localizer.r2);
//        telemetry.addData("l2", bot.localizer.l2);
//        telemetry.addData("theta", bot.localizer.theta);
//        telemetry.addData("heading", bot.getHeading());

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
