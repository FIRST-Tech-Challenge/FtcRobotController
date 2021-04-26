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

    // define our bot, telemetryHandler, and optimizer
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    Optimizer optimizer = new Optimizer();



//    AutoModuleThread autoModuleThread = new AutoModuleThread();


    @Override
    public void init() {
        // initialize the bot and telemetryHandler
        bot.init(hardwareMap);
        telemetryHandler.init(telemetry, bot);


    }

    // Start the odometry thread
    @Override
    public void start() {
        bot.startOdoThreadTele();
    }

    @Override
    public void loop() {

        // If !optimizer.show, show different telemetry than if optimizer.show
        if(!optimizer.show) {
            // move bot with gamepad1 joysticks
//            bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger);

            // update optimizer and display odometry telemetry
            optimizer.update();
            telemetryHandler.addOdometry();
            telemetry = telemetryHandler.getTelemetry();
        }else{
            // get optimizer values and telemetry them
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

        // when you press gamepad1.a, show the second set of telemetry
        if(gamepad1.a && !optimizer.show){
            optimizer.show();
        }


        // update telemetry
        telemetry.update();



    }

    // stop odometry thread
    @Override
    public void stop() {
        bot.stopOdoThread();
    }
}
