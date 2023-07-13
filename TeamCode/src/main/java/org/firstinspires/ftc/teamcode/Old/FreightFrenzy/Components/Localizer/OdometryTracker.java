package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Components.Localizer;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//5900,4900
//5700,4300
//3853
//2/1.337/pi*8092
public class OdometryTracker extends Tracker {
    private DcMotorEx encoderLeft, encoderRight, encoderBack;
    private DigitalChannel limitSwitchRight, limitSwitchLeft;
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    private double ticks_per_inch = 180800.0/94.375, ticks_per_angle = 12235.0/36, ticks_per_radian = ticks_per_angle*180/PI, width = 10.4,
            ultraRange = 35, lastUltraUpdate = 0.0, ultraLow = 0.0, time =0.0,deltaAngle;
    private double[] lastTicks = {0, 0, 0}, odomconst = {1,-1,1};
    private boolean touch, ultras, high = false;
    private final BNO055IMU imu;
    private Orientation lastAngles = null;
    public static float globalAngle = 0;
    public OdometryTracker(boolean limitSwitch, boolean ultra) {
        super();
        touch = limitSwitch;
        ultras = ultra;
        encoderLeft = (DcMotorEx) op.hardwareMap.dcMotor.get("motorRightFront");
        encoderRight = (DcMotorEx) op.hardwareMap.dcMotor.get("motorLeftFront");
        encoderBack = (DcMotorEx) op.hardwareMap.dcMotor.get("leftEncoder");
        encoderBack.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        globalAngle=0;
        deltaAngle=0;
        lastAngles=null;
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        lastAngles = new Orientation();
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        // make sure the imu gyro is calibrated before continuing.
        while (!op.isStopRequested() && !imu.isGyroCalibrated()) {
            op.sleep(50);
            op.idle();
        }
        if (limitSwitch) {
            limitSwitchLeft = op.hardwareMap.get(DigitalChannel.class, "limitSwitchLeft");
            limitSwitchRight = op.hardwareMap.get(DigitalChannel.class, "limitSwitchRight");
            limitSwitchLeft.setMode(DigitalChannel.Mode.INPUT);
            limitSwitchRight.setMode(DigitalChannel.Mode.INPUT);
        }
        if (ultra) {
            ultrasonicFront = op.hardwareMap.get(AnalogInput.class, "ultrasonicFront");
            ultrasonicBack = op.hardwareMap.get(AnalogInput.class, "ultrasonicBack");
            ultrasonicLeft = op.hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
            ultrasonicRight = op.hardwareMap.get(AnalogInput.class, "ultrasonicRight");
            ultraFront = op.hardwareMap.get(LED.class, "ultraFront");
            ultraBack = op.hardwareMap.get(LED.class, "ultraBack");
            ultraRight = op.hardwareMap.get(LED.class, "ultraRight");
            ultraLeft = op.hardwareMap.get(LED.class, "ultraLeft");
            ultraFront.enable(true);
            ultraBack.enable(true);
            ultraRight.enable(true);
            ultraLeft.enable(true);
        }
    }
    public void track() {
        time=op.getRuntime();
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
        angle += deltaAngle*180/PI;
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
        op.telemetry.addData("left",encoderLeft.getCurrentPosition());
        op.telemetry.addData("right",encoderRight.getCurrentPosition()); //left
        op.telemetry.addData("front",encoderBack.getCurrentPosition()); //right
        op.telemetry.update();
    }
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle <= -180) //If the angle is -180, it should be 180, because they are at the same point. The acceptable angles are (-180, 180]
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        globalAngle %= 360;
        if (globalAngle > 270) {
            globalAngle -= 360;
        }
        if (globalAngle < -270) {
            globalAngle += 360;
        }

        lastAngles = angles;
        return globalAngle;
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
}
