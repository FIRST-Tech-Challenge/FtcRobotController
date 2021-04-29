package globalfunctions;


import util.Geometry;

public class Constants {
    public static final double MAX_OUTTAKE_SPEED = 100*2*Math.PI; // rad/s
//
//    public static final double OUTR_SPEED_OFFSET = 0.12*MAX_OUTTAKE_SPEED;
    public static final double OUT_SPEED_OFFSET = 0.05*MAX_OUTTAKE_SPEED;

    public static final double OUTTAKE_ANGLE = 26 * Math.PI/180; // degrees -> radians
    public static final double GOAL_FROM_LEFT = 0.9; // meters
    public static final double GOAL_HEIGHT = 0.9; // meters
    public static final double POWERSHOT_HEIGHT = 0.77; // meters
    public static final double POWERSHOT_FROM_LEFT = 1.3; //meters
    public static final double DIS_BETWEEN_POWERSHOTS = 0.2; //meters

    public static final double SHOOTER_HEIGHT = 0.19; // meters
    public static final double SHOOTER_WHEEL_RADIUS = 0.05; // meters
    public static final double FIELD_LENGTH = 3.6576; //meters
    public static final double CURVATURE_TAN_THETA = Math.tan(0/294.76); // used to be 12 now 0

    public static final double CLL_GRAB = 0.2;
    public static final double CLL_OPEN = 1;
    public static final double CLR_GRAB = 1;
    public static final double CLR_OPEN = 0;
    public static final double RP_START = 0.1;
    public static final double CR_SERVO_MAX_SPEED = Math.PI/(0.13 * 3); // rad/s

    public static final double WG_LOWER_LIMIT = -5;
    public static final double WG_UPPER_LIMIT = 180;
    public static final double WG_REST_POW = 0.1;
    public static final double WG_START_POS_AUTON = -3;

    public static final double WGE_UPPER_LIMIT = 11.5; // 10.8 cm
    public static final double WGE_START = 8.5; // cm
    public static final double WGE_ACC = 1; // cm
    public static final double[] WGE_IGNORE_RANGE = {-40,0};
//    public static final double WGE_RADIUS = 2.25; //1.459 * 2.54; // cm

    public static final double ROBOT_WIDTH = 30; // meters
    public static final double ROBOT_LENGTH = 33; // meters
    public static final double ROBOT_RADIUS = Geometry.pythagoreanC(ROBOT_WIDTH, ROBOT_LENGTH)/2; // meters
    public static final double CENTER_THETA = Math.PI - Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH); // radians

    public static final int AUTOMODULE_REFRESH_RATE = 100; // hertz
    public static final int ODOMETRY_REFRESH_RATE = 100; //hertz

    public static final double NEVEREST256_TICKS = 7168;
    public static final double NEV_DEGREES_TO_TICKS = NEVEREST256_TICKS/360;
    public static final double GOBUILDA1_Ticks = 28;
    public static final double GO_DEGREES_TO_TICKS = GOBUILDA1_Ticks/360;
    public static final double GO_RAD_TO_TICKS = GOBUILDA1_Ticks/(2 * Math.PI);

    public static final double pi2 = Math.PI*2;
    public static final double halfPi = Math.PI/2;
    public static final double tfPi = halfPi+Math.PI;

    public static final double ANGLE_ACCURACY = 5; // in degrees
    public static final double POS_ACCURACY = 10; //cm

    public static final double COMPASS_START = 343; //in degrees

    public static final double TICKS_FOR_ODOMETRY =  8192; //in ticks
    public static final double ENCODER_WHEEL_RADIUS = 1.75; // in cm //1.75
    public static final double RADIUS_CENTER_TO_ENC = 2; // in cm
    public static final double DIS_BEWTEEN_ENCS = 35.4; //in cm
    public static final double CM_TO_TICKS = TICKS_FOR_ODOMETRY/(pi2*ENCODER_WHEEL_RADIUS);
    public static final double HALF_DIS_BETWEEN_ENCS = DIS_BEWTEEN_ENCS/2; // in cm
    public static final double DIS_CENTER_TO_LEFT_ENC = 16; //cm 17.4
    public static final double DIS_CENTER_TO_RIGHT_ENC = DIS_BEWTEEN_ENCS-DIS_CENTER_TO_LEFT_ENC; //cm
    public static final double DIS_CENTER_ENC_TO_CENTER = HALF_DIS_BETWEEN_ENCS-DIS_CENTER_TO_LEFT_ENC;

    public static final double[] AUTO_START = new double[]{80,21,0};
    public static final double[] TELE_START = new double[]{97,180,0};
    public static final double[] AUTO_SHOOT_POS = new double[]{115, 141, 7};
    public static final double[] AUTO_SHOOT_POS_NOT_ANGLED = new double[]{115, 141, 0};
    public static final double[] AUTO_POWERSHOT_POS = new double[]{123, 156, 0};

    public static final double UPDATE_ODOMETRY_WITH_SENSORS_RATE = 4; //hertz

    public static final double RS_POW = 1; //0.5

    public static final double SHOOT_DIS = 0.06; //m

    public static double FRICTION_ACCEL = 340; // 360 m/s^2
    public static final double MASS_OF_RING = 0.0295; //kg

    //Friction Force = 10.32 N


}
