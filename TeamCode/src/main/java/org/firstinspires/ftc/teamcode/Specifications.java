package org.firstinspires.ftc.teamcode;
//Configuration:
    /*
    Control Hub:

    Motors:
    0:LeftBack
    1:LeftForward
    2:Lift 1
    3:Lift 2

    Servos:
    0. leftLinkage
    2. leftArm
    4. claw

    Expansion Hub:
    0. roll
    1. pitch
    2. light


    Motors:
    0: RightBack
    1: RightForward
    2: Lift 3

    i2c:
    1: pinpoint odo
     */

//laser is in i2c control hub 2 and imu is in 0

public class Specifications {

    public static final String EXTENSION_MOTOR_MAIN = "lift1";
    public static final String EXTENSION_MOTOR_AUX1 = "lift2";
    public static final String EXTENSION_MOTOR_AUX2 = "lift3";


    // subsystem name

    public static final String CLAW_SERVO = "claw";
    public static final String LEFT_OUTPUT_ARM = "larm";
    public static final String RIGHT_OUTPUT_ARM = "rarm";

    public static final String EXTENSION_ARM_LEFT = "llinkage";
    public static final String EXTENSION_ARM_RIGHT = "rlinkage";
    public static final String ROLL = "roll";
    public static final String PITCH = "pitch";



    public static final String FTLF_MOTOR = "lf";
    public static final String FTRT_MOTOR = "rf";
    public static final String BKLF_MOTOR = "lb";
    public static final String BKRT_MOTOR = "rb";

    public static final String LIME_LIGHT = "lime";

    public static final String LIGHT = "light";

    public static final String PIN_POINT_ODOMETRY = "odo";

    public static final String HANGING_MOTOR = "hangingMotor";
    public static final String HANGING_MOTOR_AUX = "hangingMotorAux";

//    public static final String COLOR_SENSOR = "colorSensor";

//    public static final String LED = "led";


    public static final int CVSmoothing = 30;

    public enum NavSystem{
        IMU,
        ODOMETRY,
        MIXED
    }
    public NavSystem navSystem = NavSystem.ODOMETRY;
}
