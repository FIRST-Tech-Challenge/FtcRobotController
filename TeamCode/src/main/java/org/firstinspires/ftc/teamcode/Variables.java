package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Variables {
    //declare motors
    public static DcMotor motorFL;          //motor01
    public static DcMotor motorBL;          //motor02
    public static DcMotor motorFR;          //motor03
    public static DcMotor motorBR;          //motor04
    public static DcMotor motorSlide;
    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
        ROTATE_LEFT,
        ROTATE_RIGHT,
        ;
    }

    public static Servo servoGrabberThing;


    public static ElapsedTime runtime = new ElapsedTime();
    public static BNO055IMU imu;

    //NavX IMU stuffs below
    public static IntegratingGyroscope gyro;
    public static NavxMicroNavigationSensor navxMicro;

    public static double previousHeading = 0;
    public static double intergratedHeading = 0;
    public static double targetZ;
    public static boolean isImuCalibrated = false;


    //other variables
    public static double clicksPerRotation = 537.6;
    public static double rotationsPerMeter = 1/0.3015928947;

    public static final double Clamp = 0.5;
    public static final double Release = 0.75;

    public static final int downHeight = 0;
    public static final int collectHeight = 200;
    public static final int lowHeight = 1950;
    public static final int midHeight = 3100;
    public static final int highHeight = 4400;


    // Grbber #1 Clamp: 0.5, Release: 0.75
    // grbber #2 Clamp: 0.58, Release: 0.51
}
