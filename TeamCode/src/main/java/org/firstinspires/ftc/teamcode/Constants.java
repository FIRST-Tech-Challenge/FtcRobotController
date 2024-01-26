package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final double l1ff = 1;// com distant from axis first arm METERS
    public static final double l2ff = 0.70;// com distant from axis second arm METERS
    public static final double first_arm_weight = 1.7; // first arm weight KG
    public static final double second_arm_weight = 1.3; // second arm weight with gripper KG
    public static final double g = 9.806;
    public static final double hex_stall_current = 181;
    public static final double resistance = 12 / hex_stall_current; //volt

    public static final double gear_ratio = 80;
    public static final double hex_stall_torque = 0.182; //N * meter

    public static final double neo_Kt = hex_stall_torque / hex_stall_current;

    public static final double motorMaxVolt = 12;
    public static final double vMax = -1;//placeholder value
    public static final double vMin = -1;//placeholder value
    public static final double odometryWheelRadius=0.17; //meters
    public static final int tickPerRevolution =8192;


    public static final double TimeToAprilTagCheck = 1;
    public static final double TRACKWIDTH = 1;
    public static final double WHEEL_OFFSET = 1;
    public static final double TICKS_TO_CM = 38.862;






    final public static double cameraAngle = 0;
}
