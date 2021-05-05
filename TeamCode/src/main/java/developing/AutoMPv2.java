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
import util.Vector;

//@Disabled
@Autonomous(name="AutoMPv2", group="Auto")
public class AutoMPv2 extends LinearOpMode {

    TerraBot bot = new TerraBot();
    TelemetryHandler telemetryHandler = new TelemetryHandler();
    MotionPlanner2 yMP = new MotionPlanner2();
    MotionPlanner2 xMP = new MotionPlanner2();
    MotionPlanner2 hMP = new MotionPlanner2();

    @Override
    public void runOpMode() {
        bot.init(hardwareMap);
        telemetryHandler.init(telemetry, bot);
        bot.startOdoThreadAuto(this, false);
        telemetry.addData("Ready:", "Yes?");
        telemetry.update();
        waitForStart();

        bot.odometry.resetAll(new double[]{0,0,0});

        yMP.setPAR(0.005,0.5,0.07);
        xMP.setPAR(0.01, 0.5, 0.15);
        hMP.setPAR(0.001, 0.5, 0.1);

        yMP.setAcc(1);
        xMP.setAcc(1);
        hMP.setAcc(1);
        yMP.setTargetDis(40, 0);
        xMP.setTargetDis(0, 0);
        hMP.setTargetDis(90, 0);
        while (opModeIsActive() && !isDone()) {

            Vector disVect = new Vector(bot.odometry.x,bot.odometry.y);
            disVect.rotate(-bot.odometry.h, Vector.angle.DEGREES);
            yMP.update(disVect.y);
            xMP.update(disVect.x);
            hMP.update(bot.odometry.h);
            bot.move(yMP.getPower(), xMP.getPower(),hMP.getPower());


            //MAKE A POW VECT PLS PLS PLS PLS PLS PLS
            // TODO:
            //  MASSIVE PLS


            Sleep.trySleep(() -> Thread.sleep(10));
        }



        bot.stopOdoThread();
    }

    public boolean isDone(){
        return yMP.isDone() && xMP.isDone() && hMP.isDone();
    }
}
