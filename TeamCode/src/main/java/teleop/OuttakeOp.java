package teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import global.TerraBot;
import globalfunctions.Constants;
import globalfunctions.TelemetryHandler;
//@Disabled
@TeleOp(name = "TestOp")
public class OuttakeOp extends OpMode {
    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();


    @Override
    public void init() {
        bot.init(hardwareMap);
        bot.startOdoThreadTele();
        telemetry.addData("Ready?", "Yes!");
        telemetry.update();
        telemetryHandler.init(telemetry, bot);


    }

    @Override
    public void loop() {

        bot.moveTeleOp(-gamepad1.right_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);

        if (gamepad1.y) {
            bot.shooter.start();
        }
        bot.outtakeWithCalculations();

        bot.optimizeOdometryHeading();
        bot.updateOdometryUsingSensors();

        //2.42, 3.6576-1.08 = 2.57
//        double dis = 2.43; //m
        double targetSpeed = bot.autoAimer.targetSpeed;
//        double vtheo = targetSpeed*Constants.SHOOTER_WHEEL_RADIUS;
//        double vreal = bot.autoAimer.reverseCalcLinearSpeed(dis,0.9 - Constants.SHOOTER_HEIGHT);
//        double stheo = bot.autoAimer.velToAccel(vtheo);
//        double sreal = bot.autoAimer.velToAccel(vreal);
//        double f = stheo-sreal;
//        double sshouldapply = stheo + f;
//        double vshouldapply = bot.autoAimer.accelToVel(sshouldapply);
//        double targetSpeedReal = vshouldapply/Constants.SHOOTER_WHEEL_RADIUS;
//
        telemetry.addData("targetSpeed", targetSpeed);
//        telemetry.addData("v_theo", vtheo);
//        telemetry.addData("v_real", vreal);
//        telemetry.addData("s_theo", stheo);
//        telemetry.addData("s_real", sreal);
//        telemetry.addData("f", stheo-sreal);
//        telemetry.addData("sshouldapply", sshouldapply);
//        telemetry.addData("vshouldapply", vshouldapply);
//        telemetry.addData("targetSpeedReal", targetSpeedReal);
//        telemetry.addData("-------------------------------------", "");

        telemetryHandler.addTele(2,1,0,3,2);
        telemetry = telemetryHandler.getTelemetry();

        telemetry.update();

    }

    @Override
    public void stop() {
        bot.stopAllAutomodules();
        bot.stopOdoThread();
    }
}
