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

        if(gamepad2.y){
            bot.powerShot.start();
        }
//
        bot.update();


        telemetry.addData("OdometryX", bot.odometry.getX());
        telemetry.addData("OdometryY", bot.odometry.getY());
        telemetry.addData("Heading", bot.odometry.getTheta());
        telemetry.update();

        bot.update();

    }

    @Override
    public void stop() {
        //bot.stopOdoThreadTele();
    }
}
