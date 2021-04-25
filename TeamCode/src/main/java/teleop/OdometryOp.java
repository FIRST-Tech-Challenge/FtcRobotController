package teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import global.TerraBot;
import globalfunctions.Optimizer;
import globalfunctions.TelemetryHandler;

@Disabled
@TeleOp(name = "TestOdometry")
public class OdometryOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    Optimizer optimizer = new Optimizer();



//    AutoModuleThread autoModuleThread = new AutoModuleThread();


    @Override
    public void init() {
        bot.init(hardwareMap);
        telemetryHandler.init(telemetry, bot);


    }

    @Override
    public void start() {
        bot.startOdoThreadTele();
    }

    @Override
    public void loop() {

        if(!optimizer.show) {
//            bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger);
            optimizer.update();
            telemetryHandler.addOdometry();
            telemetry = telemetryHandler.getTelemetry();
        }else{
            double ydebA = optimizer.calcAvg(bot.odometry.Ydebug);
            double xdebA = optimizer.calcAvg(bot.odometry.Xdebug);
            double hdebA = optimizer.calcAvg(bot.odometry.Hdebug);
            double ydebM = optimizer.max(bot.odometry.Ydebug);
            double xdebM = optimizer.max(bot.odometry.Xdebug);
            double hdebM = optimizer.max(bot.odometry.Hdebug);


            telemetry.addData("avgDeltaTime", optimizer.avgDeltaTime);
            telemetry.addData("xdebA", xdebA);
            telemetry.addData("ydebA", ydebA);
            telemetry.addData("hdebA", hdebA);
            telemetry.addData("xdebM", xdebM);
            telemetry.addData("ydebM", ydebM);
            telemetry.addData("hdebM", hdebM);

            telemetry.update();
        }

        if(gamepad1.a && !optimizer.show){
            optimizer.show();
        }


        telemetry.update();



    }

    @Override
    public void stop() {
        bot.stopOdoThread();
    }
}
