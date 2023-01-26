package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

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
        ROTATE,
        ;
    }

    public static Rect focusRect = new Rect();
    public static Rect focusRect2 = new Rect(new Point(0,0), new Point(20,20));

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

    public static final double Clamp = 0.48;
    public static final double Release = 0.56;

    public static final int downHeight = 0;
    public static final int collectHeight = 200;
    public static final int lowHeight = 1850;
    public static final int midHeight = 3100;
    public static final int highHeight = 4200;


    /**OpenCV Variables please don't touch!!
     */

    public static int lowestX;
    public static int highestX;
    public static int lowestY;
    public static int highestY;

    public static double percentColor;
    public static String levelString = "one";
    public static boolean level2Capable = false;
    public static int x_resolution;
    public static int y_resolution;
    public static int focusRectWidth;
    public static int focusRectHeight;
    public static int minimumWidth;
    public static int minimumHeight;
    public static int box_width = 0;
    public static int box_height = 0;
    public static int centerX;
    public static int largestSize = 0;
    public static int largestObjectWidth = 0;
    public static int largestObjectLowestX = 640;
    public static int largestObjectHighestX = 0;


    public static int boxBL_x;
    public static int boxBL_y;



    public static int gridX = 20;
    public static int gridY = 20;
    public static int gridTotal = gridX * gridY;

    public static boolean level1Assigment = false;
    public static boolean level2 = false;
    public static boolean level2Assignment = false;
    public static boolean level3 = false;
    public static boolean level3Assignment = false;
    public static boolean level1Aligned = false;
    public static boolean level2Aligned = false;
    public static boolean level3Aligned = false;
//    public static boolean isIMURecorded = false;
//    public static boolean topOfPole = false;
    public static boolean visionAutoActivated = false;
    public static double imuHeading = 0;



    public static Rect[][] rectanglesGrid = new Rect[gridY][gridX];
    public static Rect[][] rectanglesGridDraw = new Rect[gridY][gridX];
    public static Mat[][] matsGrid = new Mat[gridY][gridX];
    public static boolean[][] identifiedBoxesBoolean = new boolean[gridY][gridX];
    public static int[][] centersX = new int[gridY][gridX];
    public static int[][] centersXDraw = new int[gridY][gridX];
    public static int[][] centersY = new int[gridY][gridX];
    public static int[][] centersYDraw = new int[gridY][gridX];




    // Grbber #1 Clamp: 0.5, Release: 0.75
    // grbber #2 Clamp: 0.41, Release: 0.52
    // grbber #3 Clamp: 0.25, Release: 0.52
}
