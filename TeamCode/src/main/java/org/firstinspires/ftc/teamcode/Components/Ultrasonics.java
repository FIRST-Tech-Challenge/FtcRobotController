package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import java.util.ArrayList;

public class Ultrasonics {
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    private double ultraRange = 35, lastUltraUpdate = -10, lastSetPos = -10, time = 0.0, robotWidth = 13, robotLength = 17.3;
    private double[] pos = {0, 0, 0}, error = {0, 0};
    private ArrayList<double[]> errorLog = new ArrayList<>();
    private boolean high = false, updated = false;

    public double[] dist = {0, 0, 0, 0};

    public Ultrasonics() {
        ultrasonicFront = op.hardwareMap.get(AnalogInput.class, "ultrasonicFront");
        ultrasonicBack = op.hardwareMap.get(AnalogInput.class, "ultrasonicBack");
        ultrasonicLeft = op.hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = op.hardwareMap.get(AnalogInput.class, "ultrasonicRight");
        ultraFront = op.hardwareMap.get(LED.class, "ultraFront");
        ultraBack = op.hardwareMap.get(LED.class, "ultraBack");
        ultraRight = op.hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = op.hardwareMap.get(LED.class, "ultraLeft");
        ultraBack.enable(true);
        ultraRight.enable(true);
        ultraFront.enable(true);
        ultraLeft.enable(true);
    }

    public void logError() {
        if(errorLog.size()==5) {
            errorLog.remove(0);
        }
        if(abs(error[0])<20&&abs(error[1])<20) {
            errorLog.add(new double[]{error[0], error[1]});
        }
    }
    public void clearError(){
        errorLog.clear();
    }
    public boolean sufficientData(){
        if(errorLog.size()<5){
            return false;
        }
        else{
            return true;
        }
    }
    public double[] averageError(){
        double[] aberage = {0,0};
        for(int i=0;i<errorLog.size();i++){
            aberage[0] += errorLog.get(i)[0];
            aberage[1] += errorLog.get(i)[1];
        }
        aberage[0]/=errorLog.size();
        aberage[1]/=errorLog.size();
        return aberage;
    }
    public boolean updateUltra(double xpos, double ypos, double angle) {
//        pos[0] = xpos;
//        pos[1] = ypos;
//        pos[2] = angle;
        updated = false;
        angle *= 180 / PI;
        time = op.getRuntime();
        if (time - lastUltraUpdate > 0.03 && !high) {
            ultraBack.enable(false);
            ultraRight.enable(false);
            ultraFront.enable(false);
            ultraLeft.enable(false);
            high = true;
        }
        if (time - lastUltraUpdate > 0.1 & high) {
            getDistance();
            double distance = dist[0] + robotWidth / 2;
            if (distance < 20 && distance > 0) {
                if (abs(angle) < 5) {
                    error[1] = -70.5 + distance - pos[1];
                    updated = true;
                } else if (abs(180 - angle) < 5) {
                    error[1] = 70.5 - distance - pos[1];
                    updated = true;

                } else if (abs(-90 - angle) < 5) {
                    error[0] = -70.5 + distance - pos[0];
                    updated = true;

                }
                if (abs(90 - angle) < 5) {
                    error[0] = 70.5 - distance - pos[0];
                    updated = true;

                }
            }
            distance = dist[1] + robotWidth / 2;
            if (distance < 20 && distance > 0) {
                if (abs(180 - angle) < 5) {
                    error[1] = -70.5 + distance - pos[1];
                    updated = true;

                } else if (abs(angle) < 5) {
                    error[1] = 70.5 - distance - pos[1];
                    updated = true;

                } else if (abs(90 - angle) < 5) {
                    error[0] = -70.5 + distance - pos[0];
                    updated = true;

                } else if (abs(-90 - angle) < 5) {
                    error[0] = 70.5 - distance - pos[0];
                    updated = true;

                }
            }
            distance = dist[2] + robotLength / 2;
            if (distance < 20 && distance > 0) {
                if (abs(180 - angle) < 5) {
                    error[0] = -70.5 + distance - pos[0];
                    updated = true;

                } else if (abs(angle) < 5) {
                    error[0] = 70.5 - distance - pos[0];
                    updated = true;

                } else if (abs(90 - angle) < 5) {
                    error[1] = -70.5 + distance - pos[1];
                    updated = true;

                } else if (abs(-90 - angle) < 5) {
                    error[1] = 70.5 - distance - pos[1];
                    updated = true;

                }
            }
            distance = dist[3] + robotLength / 2;
            if (distance < 20 && distance > 0) {
                if (abs(180 - angle) < 5) {
                    error[1] = -70.5 + distance - pos[1];
                    updated = true;

                } else if (abs(angle) < 5) {
                    error[1] = 70.5 - distance - pos[1];
                    updated = true;

                } else if (abs(90 - angle) < 5) {
                    error[0] = -70.5 + distance - pos[0];
                    updated = true;

                } else if (abs(-90 - angle) < 5) {
                    error[0] = 70.5 - distance - pos[0];
                    updated = true;

                }
            }
            if (updated) {
                logError();
            }
            if(!updated){
                clearError();
            }
        }
        if (sufficientData()&& time - lastSetPos > 1) {
            lastSetPos = time;
            return true;
        } else {
            return false;
        }
    }

    private void getDistance() {
        dist = new double[]{90.48337 * ultrasonicRight.getVoltage() - 13.12465, 90.48337 * ultrasonicLeft.getVoltage() - 13.12465
                , 90.48337 * ultrasonicFront.getVoltage() - 13.12465, 90.48337 * ultrasonicBack.getVoltage() - 13.12465};
        ultraBack.enable(true);
        ultraRight.enable(true);
        ultraFront.enable(true);
        ultraLeft.enable(true);
        lastUltraUpdate = time;
        high = false;
    }

    public Pose2d getPose2d() {
        double[] error = averageError();
        return new Pose2d(pos[0]+error[0], pos[1]+error[1], pos[2]);
    }
}
