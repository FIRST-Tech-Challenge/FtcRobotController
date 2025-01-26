package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    //Arm Reaching Positions
    public static int ARM_MAX_POSITION_OFFSET = 3000;
    public static int UP_ARM_MAX_POSITION_OFFSET = -11550;

    // PID variables
    public static float KP = 1.5F;  // Proportional gain
    public static float KI = 0.04F;  // Integral gain
    public static float KD = 0.03F;  // Derivative gain

    //servo positions
    public static int SERVO_CLOSED  = 270;
    public static int SERVO_OPEN = 120;

    //Constants to find distances
    private static final double WHEEL_DIAMETER = 104; // Wheel diameter in millimeters
    public static double COUNT_PER_REVOLUTION = 537.7 ;
    public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    public static double FRICTION_PERCENT = 0.95;
}
