package org.firstinspires.ftc.teamcode.Components;

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

public class Ultrasonics{
    private LinearOpMode op = null;
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    private double ultraRange = 35, lastUltraUpdate = 0.0, ultraLow = 0.0, time =0.0;
    private double[] pos = {0,0,0};
    private boolean high = false;
    public Ultrasonics(LinearOpMode opMode) {
        op = opMode;
            ultrasonicFront = opMode.hardwareMap.get(AnalogInput.class, "ultrasonicFront");
            ultrasonicBack = opMode.hardwareMap.get(AnalogInput.class, "ultrasonicBack");
            ultrasonicLeft = opMode.hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
            ultrasonicRight = opMode.hardwareMap.get(AnalogInput.class, "ultrasonicRight");
            ultraFront = opMode.hardwareMap.get(LED.class, "ultraFront");
            ultraBack = opMode.hardwareMap.get(LED.class, "ultraBack");
            ultraRight = opMode.hardwareMap.get(LED.class, "ultraRight");
            ultraLeft = opMode.hardwareMap.get(LED.class, "ultraLeft");
            ultraFront.enable(true);
            ultraBack.enable(true);
            ultraRight.enable(true);
            ultraLeft.enable(true);
    }

    public boolean updateUltra(double xpos, double ypos, double angle) {
        pos[0]=xpos;
        pos[1]=ypos;
        pos[2]=angle;
        time = op.getRuntime();
        if(time-ultraLow>0.06&&!high){
            ultraBack.enable(true);
            ultraRight.enable(true);
            ultraFront.enable(true);
            ultraLeft.enable(true);
            high=true;
        }
        if(time-lastUltraUpdate>0.1) {
            if (70.5 - xpos < ultraRange) {
                if (abs(angle) < 5) {
                    pos[0] = 70.5 - getDistance(ultrasonicRight, ultraRight);
                    return true;
                } else if (abs(90 - angle) < 5) {
                    pos[0] = 70.5 - getDistance(ultrasonicFront, ultraFront);
                    return true;
                } else if (abs(angle) > 175) {
                    pos[0] = 70.5 - getDistance(ultrasonicLeft, ultraLeft);
                    return true;
                } else if (abs(-90 - angle) < 5) {
                    pos[0] = 70.5 - getDistance(ultrasonicBack, ultraBack);
                    return true;
                }
            } else if (-70.5 - xpos > -ultraRange) {
                if (abs(angle) < 5) {
                    pos[0] = -70.5 - getDistance(ultrasonicLeft, ultraLeft);
                    return true;
                } else if (abs(90 - angle) < 5) {
                    pos[0] = -70.5 - getDistance(ultrasonicBack, ultraBack);
                    return true;
                } else if (abs(angle) > 175) {
                    pos[0] = -70.5 - getDistance(ultrasonicRight, ultraRight);
                    return true;
                } else if (abs(-90 - angle) < 5) {
                    pos[0] = -70.5 - getDistance(ultrasonicFront, ultraFront);
                    return true;
                }
            }
            if (70.5 - ypos < ultraRange) {
                if (abs(angle) < 5) {
                    pos[1] = 70.5 - getDistance(ultrasonicFront, ultraFront);
                    return true;
                } else if (abs(90 - angle) < 5) {
                    pos[1] = 70.5 - getDistance(ultrasonicLeft, ultraLeft);
                    return true;
                } else if (abs(angle) > 175) {
                    pos[1] = 70.5 - getDistance(ultrasonicBack, ultraBack);
                    return true;
                } else if (abs(-90 - angle) < 5) {
                    pos[1] = 70.5 - getDistance(ultrasonicRight, ultraRight);
                    return true;
                }
            } else if (-70.5 - ypos > -ultraRange) {
                if (abs(angle) < 5) {
                    pos[1] = 70.5 - getDistance(ultrasonicBack, ultraBack);
                    return true;
                } else if (abs(90 - angle) < 5) {
                    pos[1] = 70.5 - getDistance(ultrasonicRight, ultraRight);
                    return true;
                } else if (abs(angle) > 175) {
                    pos[1] = 70.5 - getDistance(ultrasonicFront, ultraFront);
                    return true;
                } else if (abs(-90 - angle) < 5) {
                    pos[1] = 70.5 - getDistance(ultrasonicLeft, ultraLeft);
                    return true;
                }
            }
        }
        return false;
    }

    public double getDistance(AnalogInput sensor, LED sensorLED) {
        double rawValue = sensor.getVoltage();
        ultraBack.enable(false);
        ultraRight.enable(false);
        ultraFront.enable(false);
        ultraLeft.enable(false);
        high=false;
        return 90.48337 * sensor.getVoltage() - 13.12465;
    }
    public Pose2d getPose2d(){
        return new Pose2d(pos[0],pos[1],pos[2]);
    }
}
