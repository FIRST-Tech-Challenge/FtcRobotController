package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //misc

    public static int visionView = 0;
    public static final double LAUNCH_HEIGHT = 0.41;
    public static final int ENCODER_TICKS_PER_REVOLUTION = 28;
    public static final double FLYWHEEL_RADIUS = 0.0765;
    public static final double GRAVITY = 9.80665;
    public static final double INCHES_PER_METER = 39.3701;


    //BEGIN Proteus Kinematics
    public static final double ROBOT_RADIUS_INCHES = 8.75;
    public static double ITERATIONS = 1;
    public static final double LAUNCHER_LENGTH = 0.24;
    public static final double LAUNCHER_VERTICAL_OFFSET = 0.085;
    public static final double BASE_LAUNCH_ANGLE = 19.50244851;

    //MUZZLE Corrections
    //keep these X and Y muzzle offsets for reference
    //public static double MUZZLE_X_OFFSET = 0.115;
    //public static double MUZZLE_Y_OFFSET = 0.09;
    //converting to polar we get:
    public static double MUZZLE_DEG_OFFSET = 90-38.0470;
    public static double MUZZLE_RAD_OFFSET = 0.906750906;
    public static double MUZZLE_RADIUS = .146;
    public static double TURRET_AXIS_OFFSET = 0.114;
    public static double TURRET_RADIUS = 0.1500;

    //END Kinematics

    public static double kpFlywheel = 0.6; //proportional constant multiplier goodish
    public static  double kiFlywheel = 1.0; //integral constant multiplier
    public static  double kdFlywheel= 0.0; //derivative constant multiplier

    // Vision
    public static int TOP_LEFT_X = 70;
    public static int TOP_LEFT_Y = 120;
    public static int BOTTOM_RIGHT_X = 180;
    public static int BOTTOM_RIGHT_Y = 180;

    public static double NORMALIZE_ALPHA = 51.0;
    public static double NORMALIZE_BETA = 261.0;

    public static double BLUR_RADIUS = 8.558558558558557;

    public static double HSV_THRESHOLD_HUE_MIN = 0.4668065215846204;
    public static double HSV_THRESHOLD_HUE_MAX = 1000;
    public static double HSV_THRESHOLD_SATURATION_MIN = 40.13039568345324;
    public static double HSV_THRESHOLD_SATURATION_MAX = 255.0;
    public static double HSV_THRESHOLD_VALUE_MIN = 109.84730100784292;
    public static double HSV_THRESHOLD_VALUE_MAX = 255.0;

    public static double MIN_CONTOUR_AREA = .1;

    public static double MIN_BLOB_SIZE = 4000;

    public static int WEBCAM_WIDTH = 320;
    public static int WEBCAM_HEIGHT = 240;

    //odometry positions all in meters
    public static double goalX = 0.9144;
    public static double goalY = 3.6576;
    public static double GOAL_RADIUS = 8;
    public static double POWER_SHOT_RADIUS = 1;
    public static double startingXOffset = 49/INCHES_PER_METER;
    public static double startingYOffset = ROBOT_RADIUS_INCHES/INCHES_PER_METER;
    public static double HEIGHT_MULTIPLIER = 1.15;
    public static double RPS_MULTIPLIER = 1.07;
    public static double MUZZLE_ANGLE_OFFSET_IN_TELE_OP = -10;
    public static int  ELBOW_ZERO_DEGREES_OFFSET = 141;
    public static double ILLEGAL_SHOOTING_DISTANCE = 1.8288;

    public static int WOBBLE_GRIPPER_CLOSED = 900;
    public static int WOBBLE_GRIPPER_STOWED = 2100;

    public static int INTAKE_TILT_SERVO_TRAVEL = 1600;
    public static int INTAKE_TILT_SERVO_HANDOFF = 1300;
    public static int INTAKE_TILT_SERVO_PICKUP = 1750;
    public static int INTAKE_TILT_FOR_OUTTAKE = 1300;
    public static int INTAKE_TILT_FOR_OUTTAKE_TOO = 1600;
    public static int INTAKE_TILT_INIT_POS = 899;

    public static int INTAKE_OUT_SERVO_OUT = 1690;
    public static int INTAKE_OUT_SERVO_IN = 900;

    public static double AUTO_INTAKE_SPEED = .8;
    public static double AUTO_INTAKE_FIRST = .65;
    public static double AUTO_INTAKE_SECOND = 1.1;

    public static double TURRET_TOLERANCE = 0.01;

    public static int LAUNCHER_TRIGGER_STOWED = 1900;
    public static int LAUNCHER_TRIGGER_SHOOT = 2200;
    public static int LAUNCHER_TRIGGER_BACK = 1780;
    public static double autoLaunchTime = .3;

    public static double __ATMEP = 1;
    public static double __ATMEP2 = -1;

    //inner conflicts
    public static double TURRET_HEADING_OFFSET = 30;
    public static double ELBOW_LEGAL_ANGLE = 35;

    public static int WALL_FOLLOW_MULTIPLIER = 50;


    public enum Target {
        NONE(0, 0, 0),
        HIGH_GOAL(0.9144, 3.6576, 0.88),
        MID_GOAL(0.9144,3.6576,.6858),
        MID_GOAL_CLASSIC(-0.9144,3.6576,.6858),
        LOW_GOAL(0.9144,3.6576,.4318),
        FIRST_POWER_SHOT(.1016,3.6576,.8),
        SECOND_POWER_SHOT(.2921,3.6576,.8),
        THIRD_POWER_SHOT(.4826,3.6576,.8);



        public double x, y, height;

        private Target(double x, double y, double height) {
            this.x = x;
            this.y = y;
            this.height = height;
        }
    }

    public enum DirectionOfTurn{
        LEFT,
        RIGHT,
        I_REALLY_DONT_CARE
    }

    public enum Position {
        //headings and elevations that are negative means don't apply them to ending position - let other behaviors control
        START(49/INCHES_PER_METER, ROBOT_RADIUS_INCHES/INCHES_PER_METER,0,0,0, 1, DirectionOfTurn.I_REALLY_DONT_CARE),
        //WOBBLE_ONE_GRAB(48/INCHES_PER_METER, (ROBOT_RADIUS_INCHES+3)/INCHES_PER_METER,0,340,0),
        WOBBLE_ONE_GRAB(49/INCHES_PER_METER, (8 + ROBOT_RADIUS_INCHES)/INCHES_PER_METER,0,0,1,1, DirectionOfTurn.I_REALLY_DONT_CARE),
        //340, 45
        ALIGNMENT_RESET(49/INCHES_PER_METER, 11*12/INCHES_PER_METER, 0,170,-1,1, DirectionOfTurn.I_REALLY_DONT_CARE),
        //turret needs to rotate counter clockwise to deposit wobble goals A and C - use intermediate turret heading of 170
        TARGET_C_1(49/INCHES_PER_METER, 10.5*12/INCHES_PER_METER, -1,90,-1,.5, DirectionOfTurn.LEFT),
        TARGET_C_2((49+7)/INCHES_PER_METER, 10.5*12/INCHES_PER_METER, -1,45+45,5,0, DirectionOfTurn.RIGHT),
        TARGET_B_1((49)/INCHES_PER_METER, 8.5*12/INCHES_PER_METER, 0,0,5,.5, DirectionOfTurn.I_REALLY_DONT_CARE), //        TARGET_B_1((49-7)/INCHES_PER_METER, 8.5*12/INCHES_PER_METER, 0,0,5,1),
        TARGET_B_2((49+7)/INCHES_PER_METER, 8*12/INCHES_PER_METER, -1,0,5,1, DirectionOfTurn.I_REALLY_DONT_CARE),
        TARGET_A_1((49+7)/INCHES_PER_METER, 7.75*12/INCHES_PER_METER, 0,90,5,.5, DirectionOfTurn.LEFT),
        TARGET_A_2((49-7)/INCHES_PER_METER, 7*12/INCHES_PER_METER, -1,45+45,5,1, DirectionOfTurn.RIGHT),
        RING_STACK(36/INCHES_PER_METER, 48/INCHES_PER_METER,-1,-1, -1,0, DirectionOfTurn.I_REALLY_DONT_CARE),
        RING_STACK_APPROACH(36/INCHES_PER_METER, (48+6+ ROBOT_RADIUS_INCHES)/INCHES_PER_METER, 180, 270,0,1, DirectionOfTurn.I_REALLY_DONT_CARE),
        //sweep needs to be very slow
        RING_STACK_SWEEPTO(36/INCHES_PER_METER, (48-10+ ROBOT_RADIUS_INCHES)/INCHES_PER_METER, 180, 270,0,1, DirectionOfTurn.I_REALLY_DONT_CARE),
        WOBBLE_TWO(24/INCHES_PER_METER, 23/INCHES_PER_METER,-1,-1, -1,1, DirectionOfTurn.I_REALLY_DONT_CARE),
        WOBBLE_TWO_APPROACH(49/INCHES_PER_METER, (26)/INCHES_PER_METER, 90, -1,-1, .5, DirectionOfTurn.I_REALLY_DONT_CARE),
        WOBBLE_TWO_GRAB (40/INCHES_PER_METER, (26)/INCHES_PER_METER, 90, 270,0,0, DirectionOfTurn.I_REALLY_DONT_CARE),
        NAVIGATE(49/INCHES_PER_METER, 6.5*12/INCHES_PER_METER,-1,-1, -1, .5, DirectionOfTurn.I_REALLY_DONT_CARE), //NAVIGATE(35/INCHES_PER_METER, 6.5*12/INCHES_PER_METER,-1,-1, -1, .5)
        LAUNCH_PREFERRED(47.2441/INCHES_PER_METER, 5.5*12/INCHES_PER_METER,180,-1, -1,0, DirectionOfTurn.I_REALLY_DONT_CARE), //LAUNCH_PREFERRED(3*12/INCHES_PER_METER, 5.5*12/INCHES_PER_METER,180,-1, -1,0)
        TEST_POS_FOR_TESTING(startingXOffset, startingYOffset+2,270,30, 10, .5, DirectionOfTurn.I_REALLY_DONT_CARE);

        public double x, y, baseHeading, launchHeading, launchElevation, launchStart;
        public DirectionOfTurn DoT;

        private Position(double x, double y, double baseHeading, double launchHeading, double launchElevation, double launchStart, DirectionOfTurn DoT) {
            this.x = x;
            this.y = y;
            this.baseHeading=baseHeading; //-1 means take no action on changing
            this.launchHeading=launchHeading; //-1 means take no action on changing
            this.launchElevation=launchElevation; //-1 means take no action on changing
            this.launchStart=launchStart;  //number 0.0 to 1.0 - progression of travel where launcher actions should start
            this.DoT = DoT;
        }
    }
}