package global;


public class Constants {
    public static final double MAX_OUTTAKE_SPEED = 100*2*Math.PI; // rad/s

    public static final double OUTTAKE_ANGLE = 20 * Math.PI/180; // degrees -> radians
    public static final double GOAL_FROM_LEFT = 0.9; // meters
    public static final double GOAL_HEIGHT = 0.9; // meters
    public static final double SHOOTER_HEIGHT = 0.25; // meters
    public static final double SHOOTER_WHEEL_RADIUS = 0.05; // meters
    public static final double FIELD_LENGTH = 3.6576; //meters

    public static final double WGE_START = 0.55;
    public static final double WGE_EXTENDED = 0.85;
    public static final double RP_START = 0.1;

    public static final double ROBOT_WIDTH = 30; // meters
    public static final double ROBOT_LENGTH = 33; // meters
    public static final double ROBOT_RADIUS = Math.sqrt(Math.pow(ROBOT_WIDTH/2, 2) + Math.pow(ROBOT_LENGTH/2, 2)); // meters
    public static final double CENTER_THETA = Math.PI - Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH); // radians

    public static final int AUTOMODULE_REFRESH_RATE = 100; // hertz
    public static final int ODOMETRY_REFRESH_RATE = 100; //hertz

    public static final double NEVEREST256_TICKS = 7168;
    public static final double NEV_DEGREES_TO_TICKS = NEVEREST256_TICKS/360;
    public static final double GOBUILDA1_Ticks = 28;
    public static final double GO_DEGREES_TO_TICKS = GOBUILDA1_Ticks/360;

    public static final double pi2 = Math.PI*2;
    public static final double ANGLE_ACCURACY = 3.5; // in degrees
    public static final double COMPASS_START = 342; //in degrees

    public static final double TICKS_FOR_ODOMETRY =  8192; //in ticks
    public static final double ENCODER_WHEEL_RADIUS = 1.75; // in cm
    public static final double RADIUS_CENTER_TO_ENC = 3; // in cm
    public static final double DIS_BEWTEEN_ENCS = 37.46; //in cm
    public static final double CM_TO_TICKS = TICKS_FOR_ODOMETRY/(pi2*ENCODER_WHEEL_RADIUS);
    public static final double HALF_DIS_BETWEEN_ENCS = DIS_BEWTEEN_ENCS/2; // in cm

}
