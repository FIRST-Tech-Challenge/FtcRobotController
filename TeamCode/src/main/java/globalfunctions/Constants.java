package globalfunctions;


import util.Geometry;

public class Constants {
    //Outtake Constants
    //--------------------------------------------------------------------------------------------------------------
    //Maximuim outtake speed possible
    public static final double MAX_OUTTAKE_SPEED = 100*2*Math.PI; // rad/s
    //Offest speed between two outtake motors
    public static final double OUT_SPEED_OFFSET = 0.05*MAX_OUTTAKE_SPEED;
    //Outtake angle
    public static final double OUTTAKE_ANGLE = 26 * Math.PI/180; // degrees -> radians
    //Height of the outtake from the ground
    public static final double SHOOTER_HEIGHT = 0.19; // meters
    //Radius of the outtake wheels
    public static final double SHOOTER_WHEEL_RADIUS = 0.05; // meters
    //Maximum cr servo speed possible
    public static final double CR_SERVO_MAX_SPEED = Math.PI/(0.13 * 3); // rad/s
    //Ring shooter power
    public static final double RS_POW = 1;


    //Field Constants
    //--------------------------------------------------------------------------------------------------------------
    //Distance the goal is from the left wall
    public static final double GOAL_FROM_LEFT = 0.9; // meters
    //The height of the goal from the ground
    public static final double GOAL_HEIGHT = 0.9; // meters
    //The distance the left powershot is from the left wall
    public static final double POWERSHOT_FROM_LEFT = 1.3; //meters
    //The height of the powershots from the ground
    public static final double POWERSHOT_HEIGHT = 0.77; // meters
    //The distance between powershots
    public static final double DIS_BETWEEN_POWERSHOTS = 0.2; //meters
    //The length of the field
    public static final double FIELD_LENGTH = 3.6576; //meters
    //The curvature slope (not used)
    public static final double CURVATURE_TAN_THETA = Math.tan(0/294.76); // used to be 12 now 0


    //Robot Constants
    //--------------------------------------------------------------------------------------------------------------
    //Position of the left claw grabbing
    public static final double CLL_GRAB = 0.2;
    //Position of the left claw open
    public static final double CLL_OPEN = 1;
    //Position of the right claw grabbing
    public static final double CLR_GRAB = 1;
    //Position of the right claw open
    public static final double CLR_OPEN = 0;
    //Position of the left front closed
    public static final double FLS_CLOSED = 0.12;
    //Position of the left front open
    public static final double FLS_OPEN = 0.62;
    //Position of the right front closed
    public static final double FRS_CLOSED = 0.02;
    //Position of the right front open
    public static final double FRS_OPEN = 0.7;
    //Wobble goal lower limit
    public static final double WG_LOWER_LIMIT = -5; //deg
    //Wobble goal upper limit
    public static final double WG_UPPER_LIMIT = 180; //deg
    //Wobble goal rest power
    public static final double WG_REST_POW = 0.1;
    //Wobble goal extender upper limit
    public static final double WGE_UPPER_LIMIT = 11.5; // cm
    //Start position of wobble goal extender
    public static final double WGE_START = 8.5; // cm
    //The accuracy of the wobble goal extender
    public static final double WGE_ACC = 1; // cm
    //The range in which the wobble goal extender should not move
    public static final double[] WGE_IGNORE_RANGE = {-40,0}; //deg
    //The width of the robot
    public static final double ROBOT_WIDTH = 30; // cm
    //The length of the robot
    public static final double ROBOT_LENGTH = 33; // cm
    //The radius of the circumcircle of the robot
    public static final double ROBOT_RADIUS = Geometry.pythagoreanC(ROBOT_WIDTH, ROBOT_LENGTH)/2; // cm
    //The angle that the robot corners make to the front of the robot
    public static final double CENTER_THETA = Math.PI - Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH); // radians

    //Motor Constants
    //--------------------------------------------------------------------------------------------------------------
    //Ticks in big boy
    public static final double NEVEREST256_TICKS = 7168;
    //Degrees to ticks for big boy
    public static final double NEV_DEGREES_TO_TICKS = NEVEREST256_TICKS/360;
    //Outtake motor ticks
    public static final double GOBUILDA1_Ticks = 28;
    //Degrees to ticks for outtake
    public static final double GO_DEGREES_TO_TICKS = GOBUILDA1_Ticks/360;
    //Radians to ticks for outtake
    public static final double GO_RAD_TO_TICKS = GOBUILDA1_Ticks/(2 * Math.PI);

    //PI Constants
    //--------------------------------------------------------------------------------------------------------------
    //2*pi
    public static final double pi2 = Math.PI*2;
    //0.5*pi
    public static final double halfPi = Math.PI/2;
    //0.75*pi
    public static final double tfPi = halfPi+Math.PI;

    //Odometry Constants
    //--------------------------------------------------------------------------------------------------------------
    //Ticks for odometry
    public static final double TICKS_FOR_ODOMETRY =  8192; //in ticks
    //Radius of encoder wheels
    public static final double ENCODER_WHEEL_RADIUS = 1.75; // in cm
    //Distance between the right and left encoders
    public static final double DIS_BEWTEEN_ENCS = 35.4; //in cm
    //Half the distance between right and left encoders
    public static final double HALF_DIS_BETWEEN_ENCS = DIS_BEWTEEN_ENCS/2; // in cm
    //Cm to ticks for odometry
    public static final double CM_TO_TICKS = TICKS_FOR_ODOMETRY/(pi2*ENCODER_WHEEL_RADIUS);
    //Center Encoder to left encoder
    public static final double DIS_CENTER_TO_LEFT_ENC = 16; //cm
    //Center Encoder to right encoder
    public static final double DIS_CENTER_TO_RIGHT_ENC = DIS_BEWTEEN_ENCS-DIS_CENTER_TO_LEFT_ENC; //cm
    //Distance of the center encoder wheel to the center of robot
    public static final double DIS_CENTER_ENC_TO_CENTER = HALF_DIS_BETWEEN_ENCS-DIS_CENTER_TO_LEFT_ENC;
    //Distance of the center encoder wheel to the center of robot
    public static final double DIS_CENTER_ENC_TO_CENTER_HEIGHT = 2; //cm


    //AutoAimer Constants
    //--------------------------------------------------------------------------------------------------------------
    //Distance rings are touching outtake wheels
    public static final double SHOOT_DIS = 0.06; //m
    //Acceleration due to friction
    public static double FRICTION_ACCEL = 340; // 360 m/s^2
    //Mass of a ring
    public static final double MASS_OF_RING = 0.0295; //kg

    //Refresh Constants
    //--------------------------------------------------------------------------------------------------------------
    //The refresh rate of Robot Functions
    public static final int ROBOT_FUNCTIONS_REFRESH_RATE = 100; //hertz
    //The refresh rate of Odometry
    public static final int ODOMETRY_REFRESH_RATE = 100; //hertz
    //The refresh rate of updating odoemtry with sensors
    public static final double UPDATE_ODOMETRY_WITH_SENSORS_RATE = 4; //hertz


    //Accuracy Constants
    //--------------------------------------------------------------------------------------------------------------
    //Accuracy of heading for robot
    public static final double ANGLE_ACCURACY = 5; // in degrees
    //Accuracy of position for robot
    public static final double POS_ACCURACY = 10; //cm

    //Auton Constants
    //--------------------------------------------------------------------------------------------------------------
    //Start position of auton
    public static final double[] AUTO_START = new double[]{78,21,0};
    //Wobble start in autonomous
    public static final double WG_START_POS_AUTON = -3; //deg









}
