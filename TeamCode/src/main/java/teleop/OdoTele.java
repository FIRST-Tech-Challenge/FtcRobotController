package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import autofunctions.Odometry;
import global.TerraBot;
import telefunctions.Cycle;
import telefunctions.ServoController;
import telefunctions.TeleThread;

@TeleOp(name = "OdoTele V1")
public class OdoTele extends OpMode {

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
        bot.startOdoThread();
    }

    @Override
    public void loop() {

        double forward = -gamepad1.right_stick_y/2;
        double strafe = gamepad1.right_stick_x/2;
        double turn = -gamepad1.left_stick_x/2;

        bot.move(forward, strafe, turn);


        telemetry.addData("Heading", bot.getHeading());
        telemetry.addData("deltaX", bot.odometry.testx);
        telemetry.addData("deltaY", bot.odometry.testy);
        telemetry.addData("OdometryX", bot.odometry.getX());
        telemetry.addData("OdometryY", bot.odometry.getY());
        telemetry.addData("Heading", bot.getHeading());
        telemetry.addData("sketch angle", bot.odometry.thetaSketch);
        telemetry.update();

        bot.update();

    }

    @Override
    public void stop() {
        bot.stopOdoThread();
    }
}
