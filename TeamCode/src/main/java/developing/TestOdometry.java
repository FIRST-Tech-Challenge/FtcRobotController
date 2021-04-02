package developing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestOdometry")
public class TestOdometry extends OpMode {
    TestRobot bot = new TestRobot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();

    Optimizer optimizer = new Optimizer();

//    AutoModuleThread autoModuleThread = new AutoModuleThread();


    @Override
    public void init() {
        bot.init(hardwareMap);


    }

    @Override
    public void start() {
        bot.startOdoThreadTele();
    }

    @Override
    public void loop() {

        if(!optimizer.show) {

            double forward = -gamepad1.right_stick_y;
            double strafe = gamepad1.right_stick_x;
            double turn = -gamepad1.left_stick_x;

            bot.moveTeleOp(forward, strafe, turn);
            optimizer.update();
            telemetry = telemetryHandler.addOdometry(telemetry, bot);
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
