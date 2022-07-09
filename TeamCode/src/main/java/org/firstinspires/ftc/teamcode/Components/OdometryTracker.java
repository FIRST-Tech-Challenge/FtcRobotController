package org.firstinspires.ftc.teamcode.Components;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

public class OdometryTracker extends tracker {
    private LinearOpMode op = null;
    private DcMotorEx encoderLeft, encoderRight, encoderBack;
    private DigitalChannel limitSwitchRight, limitSwitchLeft;
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    private double ticks_per_inch = 180800.0/94.375, ticks_per_angle = 12235.0/36, ticks_per_radian = ticks_per_angle*180/PI, width = 10.4, ultraRange = 35;
    private double[] lastTicks = {0, 0, 0}, odomconst = {1,-1,1};
    private boolean touch, ultras;
    //1223500 ==>3600 deg   180800==>94.375
    //add set true states for ultra
    public OdometryTracker(LinearOpMode opMode, boolean limitSwitch, boolean ultra) {
        super(opMode);
        op = opMode;
        touch = limitSwitch;
        ultras = ultra;
        encoderLeft = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightFront");
        encoderRight = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        encoderBack = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightBack");
        if (limitSwitch) {
            limitSwitchLeft = opMode.hardwareMap.get(DigitalChannel.class, "limitSwitchLeft");
            limitSwitchRight = opMode.hardwareMap.get(DigitalChannel.class, "limitSwitchRight");
            limitSwitchLeft.setMode(DigitalChannel.Mode.INPUT);
            limitSwitchRight.setMode(DigitalChannel.Mode.INPUT);
        }
        if (ultra) {
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
    }

    public void track() {
        double[] nowTicks = {odomconst[0]*encoderLeft.getCurrentPosition(), odomconst[1]*encoderRight.getCurrentPosition(),
                odomconst[2]*encoderBack.getCurrentPosition()};
        double[] deltaTicks = {nowTicks[0] - lastTicks[0], nowTicks[1] - lastTicks[1], nowTicks[2] - lastTicks[2]};
        lastTicks = nowTicks;
        double deltaAngle = (deltaTicks[0] - deltaTicks[1]) / ticks_per_radian;
        //0 is front, 1 is side
        if(angle==0){
            angle=0.0000001;
        }
        if(deltaAngle==0){
            deltaAngle=0.00000000001;
        }
        if(deltaTicks[0]-deltaTicks[1]==0){
            deltaTicks[1]+=0.01;
        }
        double[] turningRadius = {width / 2 * ((deltaTicks[0]/ticks_per_inch + deltaTicks[1]/ticks_per_inch) / (deltaTicks[0]/ticks_per_inch - deltaTicks[1]/ticks_per_inch)),
                deltaTicks[2]/ticks_per_inch / deltaAngle};
        double sinsin = sin(angle * PI / 180) * sin(deltaAngle), coscos = cos(angle * PI / 180) * (1 - cos(deltaAngle)),
                cossin = cos(angle * PI / 180) * sin(deltaAngle), sincos = sin(angle * PI / 180) * (1 - cos(deltaAngle));
        double deltaX = turningRadius[0] * (sinsin + coscos) + turningRadius[1] * (cossin + sincos);
        double deltaY = turningRadius[0] * (cossin - sincos) + turningRadius[1] * (-sinsin + coscos);
        angle = (nowTicks[0] - nowTicks[1])/ticks_per_angle %360;
        xpos += deltaX;
        ypos += deltaY;
        if (touch) {
            updateTouch();
        }
        if (ultras) {
            updateUltra();
        }
        op.telemetry.addData("xpos",xpos);
        op.telemetry.addData("ypos",ypos);
        op.telemetry.addData("angle",angle);
        op.telemetry.addData("xpos",turningRadius[0]);
        op.telemetry.addData("ypos",turningRadius[0]*(cossin - sincos));
        op.telemetry.addData("angle",deltaY/ticks_per_inch);
        op.telemetry.update();
    }

    private void updateTouch() {
        if (limitSwitchRight.getState()) {
            if (abs(angle) < 5) {
                xpos = 70.5;
            } else if (abs(90 - angle) < 5) {
                ypos = -70.5;
            } else if (abs(angle) > 175) {
                xpos = -70.5;
            } else if (abs(-90 - angle) < 5) {
                ypos = 70.5;
            }
        }
        if (limitSwitchLeft.getState()) {
            if (abs(angle) < 5) {
                xpos = -70.5;
            } else if (abs(90 - angle) < 5) {
                ypos = 70.5;
            } else if (abs(angle) > 175) {
                xpos = 70.5;
            } else if (abs(-90 - angle) < 5) {
                ypos = -70.5;
            }
        }
    }

    private void updateUltra() {
        if (70.5 - xpos < ultraRange) {
            if (abs(angle) < 5) {
                xpos = 70.5 - getDistance(ultrasonicRight, ultraRight);
            } else if (abs(90 - angle) < 5) {
                xpos = 70.5 - getDistance(ultrasonicFront, ultraFront);
            } else if (abs(angle) > 175) {
                xpos = 70.5 - getDistance(ultrasonicLeft, ultraLeft);
            } else if (abs(-90 - angle) < 5) {
                xpos = 70.5 - getDistance(ultrasonicBack, ultraBack);
            }
        } else if (-70.5 - xpos > -ultraRange) {
            if (abs(angle) < 5) {
                xpos = -70.5 - getDistance(ultrasonicLeft, ultraLeft);
            } else if (abs(90 - angle) < 5) {
                xpos = -70.5 - getDistance(ultrasonicBack, ultraBack);
            } else if (abs(angle) > 175) {
                xpos = -70.5 - getDistance(ultrasonicRight, ultraRight);
            } else if (abs(-90 - angle) < 5) {
                xpos = -70.5 - getDistance(ultrasonicFront, ultraFront);
            }
        }
        if (70.5 - ypos < ultraRange) {
            if (abs(angle) < 5) {
                ypos = 70.5 - getDistance(ultrasonicFront, ultraFront);
            } else if (abs(90 - angle) < 5) {
                ypos = 70.5 - getDistance(ultrasonicLeft, ultraLeft);
            } else if (abs(angle) > 175) {
                ypos = 70.5 - getDistance(ultrasonicBack, ultraBack);
            } else if (abs(-90 - angle) < 5) {
                ypos = 70.5 - getDistance(ultrasonicRight, ultraRight);
            }
        } else if (-70.5 - ypos > -ultraRange) {
            if (abs(angle) < 5) {
                ypos = 70.5 - getDistance(ultrasonicBack, ultraBack);
            } else if (abs(90 - angle) < 5) {
                ypos = 70.5 - getDistance(ultrasonicRight, ultraRight);
            } else if (abs(angle) > 175) {
                ypos = 70.5 - getDistance(ultrasonicFront, ultraFront);
            } else if (abs(-90 - angle) < 5) {
                ypos = 70.5 - getDistance(ultrasonicLeft, ultraLeft);
            }
        }
    }

    public double getDistance(AnalogInput sensor, LED sensorLED) {
        double rawValue = sensor.getVoltage();
        sensorLED.enable(false);
        return 90.48337 * sensor.getVoltage() - 13.12465;
    }
}
