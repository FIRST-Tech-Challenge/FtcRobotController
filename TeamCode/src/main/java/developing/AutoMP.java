package developing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import auto.AutoHandler;
import global.TerraBot;
import globalfunctions.Sleep;
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
        bot.startOdoThreadAuto(this, false);
        telemetry.addData("Ready:", "Yes?");
        telemetry.update();
        waitForStart();
//        globalTime.reset();
        boolean isExecuting = true;
        double distance = 0.3;
        bot.odometry.resetAll(new double[]{0,0,0});
        double oldypos = 0;
        double oldtime = 0;
        globalTime.reset();
        motionPlanner.setAcc(0.01);

        //MOI = 0.305  Torque = 5.76 Nm  a = 18.8 rad/s^2 , 3 rev/s^2
        //2, 9.4, 0.4 acc = 0.01
        //3.5 , 8, 0.4 acc = 0.01
        //
        motionPlanner.setFML(0.5, 9.4, 0.4);
        while (opModeIsActive() && isExecuting) {

            double ypos = (bot.odometry.y/100);
            double deltaYpos = ypos - oldypos;
            oldypos = ypos;

            double curtime = globalTime.seconds();
            double deltaTime = curtime - oldtime;
            oldtime = curtime;


            double yvel = deltaYpos/deltaTime;

            double yerr = distance - ypos;
            motionPlanner.update(yerr, yvel);
            double ypow = motionPlanner.getPower();

            bot.move(ypow, 0,0);

            telemetry.addData("ypos", ypos);
            telemetry.addData("yerr", yerr);
            telemetry.addData("deltaYpos", deltaYpos);
            telemetry.addData("yvel", yvel);
            telemetry.addData("ypow", ypow);
            telemetry.addData("curTime", curtime);

            telemetry.addData("a", motionPlanner.a);
            telemetry.addData("b", motionPlanner.b);
            telemetry.addData("c", motionPlanner.c);

//            if(motionPlanner.isDone(yerr)){
//                isExecuting = false;
//            }


            telemetryHandler.addOdometry(0);
            telemetry.update();

            Sleep.trySleep(() -> Thread.sleep(10));
        }



        bot.stopOdoThread();
    }
}
