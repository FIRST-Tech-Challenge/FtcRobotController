package teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
@Disabled
@TeleOp(name = "MoveOp V1")
public class MoveOp extends OpMode {

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
        //bot.startOdoThreadTele();
    }

    @Override
    public void loop() {

        double forward = -gamepad1.right_stick_y;
        double strafe = gamepad1.right_stick_x;
        double turn = -gamepad1.left_stick_x;

        bot.move(forward, strafe, turn);

//
//        telemetry.addData("Heading", bot.getHeading());
//        telemetry.addData("deltaX", bot.odometry.testx);
//        telemetry.addData("deltaY", bot.odometry.testy);
//        telemetry.addData("OdometryX", bot.odometry.getX());
//        telemetry.addData("OdometryY", bot.odometry.getY());
//        telemetry.addData("Heading", bot.getHeading());
//        telemetry.addData("sketch angle", bot.odometry.thetaEnc);
//        telemetry.update();

//        bot.update();

    }

    @Override
    public void stop() {
        //bot.stopOdoThreadTele();
    }
}
