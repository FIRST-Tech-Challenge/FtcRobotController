package org.firstinspires.ftc.team6220_CENTERSTAGE;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

@Config
public class Constants {
  
    /* Constants used in TeleOp */

    public static final double TURN_STICK_DEADZONE = 0.01;
    public static final double TURN_POWER_MULTIPLIER = 1.0;
    public static final double DRIVE_POWER_X_MULTIPLIER = 1.0;
    public static final double DRIVE_POWER_Y_MULTIPLIER = 0.7;
    public static final double TELEOP_MIN_HEADING_ACCURACY = 5.0; // degrees off from target
    public static final double SLOWMODE_MULTIPLIER = 0.3;
    public static final double INTAKE_POWER_MULTIPLIER = 0.8;
    public static final double DRONE_SERVO_PRIMED_POS = 1;
    public static final double DRONE_SERVO_LAUNCHING_POS = 0.3;

    public static final int[] SLIDE_POSITIONS =  {
      0,
      16,
      32,
      64,
    };
    public static final double SLIDE_RETURN_MOTOR_POWER_MUL = 0.6;
    public static final double SLIDE_RETURN_MOTOR_POWER_OFFSET = -0.05;
    public static final double SLIDE_P_GAIN = 0.01;

    public static final double[] INTAKE_POSITIONS = {
        0.325,
        0.34,
        0.36,
        0.39,
        0.41,
        0.81,
    };

  
    /* Constants used in AutoFramework */

    // /!\ old x and y factor calibration was for incorrect value of 537.7;
    public static final double TICKS_PER_REVOLUTION = 537.6;
    public static final double GEAR_RATIO = 1.0;
    public static final double WHEEL_DIAMETER = 3.78; // inches
    public static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (WHEEL_DIAMETER * Math.PI);
    public static final double INCHES_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    public static final double INCHES_PER_TICK = INCHES_PER_REVOLUTION / TICKS_PER_REVOLUTION;

    // /!\ x and y factors are outdated; was made using incorrect ticks per revolution
    public static final double AUTO_X_FACTOR = 1.0867924528301887;
    public static final double AUTO_Y_FACTOR = 0.9411764705882353;
    public static final double ROBOT_AUTO_SPEED = 0.5;

    // degrees off from target
    public static final double AUTO_MIN_HEADING_ACCURACY = 5.0;

  
  
    /* other constants */
  
    // Color ranges for OpenCV detection
    public static final Scalar RED_COLOR_DETECT_MAX_HSV = new Scalar(10, 255, 255);
    public static final Scalar RED_COLOR_DETECT_MIN_HSV = new Scalar(0, 65, 95);
    public static final Scalar BLUE_COLOR_DETECT_MAX_HSV = new Scalar(140, 255, 255);
    public static final Scalar BLUE_COLOR_DETECT_MIN_HSV = new Scalar(90, 65, 25);
    public static final Scalar borderColors = new Scalar(255,170,0);
    public static final int CAMERA_WIDTH = 1920;
    public static final int CAMERA_HEIGHT = 1080;
    //public static final int CAMERA_WIDTH = 1280; // << for smol camera
    //public static final int CAMERA_HEIGHT = 960;
    public static final int SLICE_DIV_1 = CAMERA_WIDTH/3;
    public static final int SLICE_DIV_2 = (CAMERA_WIDTH/3)*2;
    // Sets blur size for gaussian blur in color detection
    public static final Size BLUR_SIZE = new Size(5, 5);

}
