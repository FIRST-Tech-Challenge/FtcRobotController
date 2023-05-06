package org.firstinspires.ftc.teamcode.roadrunner.util;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.String.valueOf;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LED;

import java.util.ArrayList;

public class Ultrasonics {
    //private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    //private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    private AnalogInput ultrasonicLeft, ultrasonicRight;
    private LED ultraLeft, ultraRight;
    public double ultraRange = 35, lastUltraUpdate = -10, lastSetPos = -10, time = 0.0, robotWidth = 13, robotLength = 15;
    //x,y,a
    private double[] pos = {0, 0, 0}, error = {0, 0};
    public ArrayList<double[]> errorLog = new ArrayList<>();
    private boolean high = false;
    public int updated = 0, updatedto = 0;
    private Pose2d pose2d;

    public double[] dist = {0, 0,0};
    private LineRegressionFilter rFilter,lFilter,aFilter;

    public Ultrasonics(double trust, int histLength) {
//        ultrasonicFront = op.hardwareMap.get(AnalogInput.class, "ultrasonicFront");
//        ultrasonicBack = op.hardwareMap.get(AnalogInput.class, "ultrasonicBack");
        ultrasonicLeft = op.hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = op.hardwareMap.get(AnalogInput.class, "ultrasonicRight");
//        ultraFront = op.hardwareMap.get(LED.class, "ultraFront");
//        ultraBack = op.hardwareMap.get(LED.class, "ultraBack");
        ultraRight = op.hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = op.hardwareMap.get(LED.class, "ultraLeft");
//        ultraBack.enable(true);
        ultraRight.enable(true);
//        ultraFront.enable(true);
        ultraLeft.enable(true);
        logger.createFile("Ultrasonics","error0, error1");
        rFilter = new LineRegressionFilter(trust,histLength);
        lFilter = new LineRegressionFilter(trust,histLength);
        aFilter = new LineRegressionFilter(trust,histLength);
    }

    public boolean updateUltra(double xpos, double ypos, double angle) {
        updatedto = 0;
        pos[0] = xpos;
        pos[1] = ypos;
        pos[2] = angle;
        updated = 0;
        angle *= 180 / PI;
        time = op.getRuntime();
        if (angle > 180) {
            angle -= 360;
        }
        if (angle < -180) {
            angle += 360;
        }
        if (time - lastUltraUpdate > 0.05 && !high) {
//            ultraBack.enable(false);
            ultraRight.enable(false);
//            ultraFront.enable(false);
            ultraLeft.enable(false);
            high = true;
        }
        if (time - lastUltraUpdate > 0.1 & high) {
            error[0] = 0;
            error[1] = 0;
            updateDistance();
            double distance = (dist[0]+dist[1])/2;
        }
        return false;
    }

    private void updateDistance() {
        dist = new double[]{rFilter.regressedDist(90.48337 * ultrasonicRight.getVoltage() - 12.62465),
                lFilter.regressedDist(90.48337 * ultrasonicLeft.getVoltage() - 12.62465),
        aFilter.regressedDist(asin((dist[0]-dist[1])/robotWidth))};
//                , 90.48337 * ultrasonicFront.getVoltage() - 13.12465, - 13.12465};
//        ultraBack.enable(true);
        ultraRight.enable(true);
//        ultraFront.enable(true);
        ultraLeft.enable(true);
        lastUltraUpdate = time;
        high = false;
    }
    public double[] getDistance(){
        return dist;
    }
    public Pose2d getPose2d() {
        return pose2d;
    }

}
