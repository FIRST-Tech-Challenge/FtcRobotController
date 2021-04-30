package developing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import auto.AutoHandler;
import global.TerraBot;
import globalfunctions.TelemetryHandler;

@Disabled
@Autonomous(name="AutoMP", group="Auto")
public class AutoMP extends LinearOpMode {

    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    ElapsedTime globalTime = new ElapsedTime();
    MotionPlanner motionPlanner = new MotionPlanner();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        telemetryHandler.init(telemetry, bot);
        bot.startOdoThreadAuto(this);
        telemetry.addData("Ready:", "Yes?");
        telemetry.update();
        waitForStart();
        globalTime.reset();
        boolean isExecuting = true;
        double distance = 0.3;
        motionPlanner.setTarget(distance, 0);
        bot.odometry.resetAll(new double[]{0,0,0});
        double oldypos = 0;
        double oldtime = 0;
        while (opModeIsActive() && isExecuting) {

            double ypos = (bot.odometry.y/100);
            double deltaYpos = ypos - oldypos;
            oldypos = ypos;

            double curtime = globalTime.seconds();
            double deltaTime = curtime - oldtime;
            oldtime = curtime;


            double yvel = deltaYpos/deltaTime;

            double yerr = distance - (bot.odometry.y/100);
            double ypow = motionPlanner.getPow(yerr, yvel,curtime);

            bot.move(ypow,0,0);

            telemetry.addData("ypos", ypos);
            telemetry.addData("yerr", yerr);
            telemetry.addData("yvel", yvel);
            telemetry.addData("ypow", ypow);


            telemetryHandler.addOdometry(1);
            telemetry.update();
        }



        bot.stopOdoThread();
    }
}
