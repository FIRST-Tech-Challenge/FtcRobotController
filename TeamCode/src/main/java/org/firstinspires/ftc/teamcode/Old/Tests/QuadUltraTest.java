package org.firstinspires.ftc.teamcode.Old.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.floor;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.util.LineRegressionFilter;

import java.util.ArrayList;

@Config
@TeleOp(name= "RAAAAAAAAAAAAAAAAAAAAAAAAHHH")
public class QuadUltraTest extends LinearOpMode {
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    public static int length = 5;
    public static double trust =0.5;
    double lastReadTime =0, lastAvgTime = 0;
    ArrayList<Double> rawHist,filHist;
    @Override
    public void runOpMode(){
//        logger.createFile("/RobotLogs/GeneralRobot", "Runtime    Component               " +
//                "Function                        Action");
        BasicRobot robot = new BasicRobot(this,false);
        LineRegressionFilter rfilter = new LineRegressionFilter(trust,length);
        LineRegressionFilter lfilter = new LineRegressionFilter(trust,length);
        LineRegressionFilter afilter = new LineRegressionFilter(trust,length);

        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = hardwareMap.get(AnalogInput.class, "ultrasonicRight");

//        ultraRight = hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = hardwareMap.get(LED.class, "ultraLeft");
        rawHist = new ArrayList<>();
        filHist = new ArrayList<>();
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        while(opModeIsActive()){
            if(getRuntime()-lastReadTime>0.1){
//                ultraRight.enable(false);
                ultraLeft.enable(true);
                            double rightRaw = 90.48337 * ultrasonicRight.getVoltage() - 12.62465;
                double leftRaw = 90.48337 * ultrasonicLeft.getVoltage() - 12.42465;
            double rightFilter = rfilter.regressedDist(rightRaw);
                double leftFilter = lfilter.regressedDist(leftRaw);
//            op.telemetry.addData("rightRaw", rightRaw);
                op.telemetry.addData("leftraw","%.2f", leftRaw);
//                logger.logNoTime("/RobotLogs/GeneralRobot", ""+leftRaw+","+leftFilter);
            op.telemetry.addData("rightFil", rightFilter);
                op.telemetry.addData("leftFil","%.2f", leftFilter);

//            op.telemetry.addData("right", ultraRight.isLightOn());
                op.telemetry.addData("left", ultraLeft.isLightOn());
                op.telemetry.addData("angle", asin((rightFilter-leftFilter)/8.5)*180/PI);
                telemetry.update();
                lastReadTime = getRuntime();

                rawHist.add(leftRaw);
                filHist.add(afilter.regressedDist(rightRaw));
            }
            else{
//                ultraRight.enable(true);

                ultraLeft.enable(false);
            }
            if(getRuntime()-lastAvgTime>5){
                logger.logNoTime("/RobotLogs/GeneralRobot", ""+getMean(rawHist)+","+getDev(rawHist)+","+getMean(filHist)+","+getDev(filHist));
                lastAvgTime =getRuntime();
                rawHist.clear();
                filHist.clear();
            }


        }
        stop();
    }
    public double getMean(ArrayList<Double> data){
        double n = 0;
        for(int i=0;i<data.size();i++){
            n+= data.get(i);
        }
        return n/(data.size());
    }
    public double getDev(ArrayList<Double> data){
        double mean = getMean(data);
        double n = 0;
        for(int i=0;i<data.size();i++){
            n+= Math.pow((Math.abs(data.get(i) - mean)), 2);
        }
        return Math.sqrt(n/data.size());
    }



}
