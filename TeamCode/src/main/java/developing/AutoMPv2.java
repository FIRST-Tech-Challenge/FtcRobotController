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
@Autonomous(name="AutoMPv2", group="Auto")
public class AutoMPv2 extends LinearOpMode {

    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    ElapsedTime globalTime = new ElapsedTime();
    MotionPlanner2 motionPlanner2 = new MotionPlanner2();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        telemetryHandler.init(telemetry, bot);
        bot.startOdoThreadAuto(this, false);
        telemetry.addData("Ready:", "Yes?");
        telemetry.update();
        waitForStart();

        boolean isExecuting = true;
        double distance = 30;
        bot.odometry.resetAll(new double[]{0,0,0});
        double oldpos = 0;
        double oldtime = 0;
        globalTime.reset();


        motionPlanner2.setPAR(0.001,1,0);
        motionPlanner2.setTargetDis(distance);

        while (opModeIsActive() && isExecuting) {

            double pos = (bot.odometry.y);
            double deltaYpos = pos - oldpos;
            oldpos = pos;

            double curtime = globalTime.seconds();
            double deltaTime = curtime - oldtime;
            oldtime = curtime;


            double vel = deltaYpos/deltaTime;
            motionPlanner2.update(pos, vel);
            double pow = motionPlanner2.getPower();

            bot.move(pow, 0,0);

            telemetry.addData("ypos", pos);
            telemetry.addData("deltaYpos", deltaYpos);
            telemetry.addData("yvel", vel);
            telemetry.addData("ypow", pow);
            telemetry.addData("curTime", curtime);

            telemetry.addData("pow", motionPlanner2.getPower());
            telemetry.addData("v(s)", motionPlanner2.VofS(pos));
            telemetry.addData("restPow", motionPlanner2.getRestPow(pos));

//            if(motionPlanner.isDone(dis-ypos)){
//                isExecuting = false;
//            }


            telemetryHandler.addOdometry(0);
            telemetry.update();

            Sleep.trySleep(() -> Thread.sleep(10));
        }



        bot.stopOdoThread();
    }
}
