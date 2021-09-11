package org.firstinspires.ftc.teamcode.robots.UGBot.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //misc

    public static int visionView = 0;
    public static double LAUNCH_HEIGHT = 0.41;
    public static final int ENCODER_TICKS_PER_REVOLUTION = 28;
    public static final double FLYWHEEL_RADIUS = 0.0765;
    public static final double GRAVITY = 9.80665;
    public static final double INCHES_PER_METER = 39.3700787;
    public static Alliance ALLIANCE = Alliance.RED;
    //public static int ALLIANCE_INT_MOD = (ALLIANCE == Alliance.RED ? 1 : -1); //todo don't know what this is, evaluates only on load, not in a loop
    public static int ALLIANCE_INT_MOD = 1;
    public static boolean isInner = false;

    public static double LINE_DETECTION_THRESHHOLD = .7;


    //BEGIN Proteus Kinematics
    public static double ROBOT_RADIUS_INCHES = 8.5;
    public static double ITERATIONS = 10;
    public static double LAUNCHER_LENGTH = 9 / INCHES_PER_METER;
    public static double LAUNCHER_VERTICAL_OFFSET = 4.5 / INCHES_PER_METER;
    public static double BASE_LAUNCH_ANGLE = 19.50244851;
    public static double ELBOW_TARGET_ANGLE = 50;
    public static int elbowMaxSafetyOffset = 70;

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

    public static double K_AMP_ALPHA = .2;

    // Vision
    public static int TOP_LEFT_X_RED = 70;
    public static int TOP_LEFT_Y_RED = 160;
    public static int BOTTOM_RIGHT_X_RED = 180;
    public static int BOTTOM_RIGHT_Y_RED = 230;

    public static int TOP_LEFT_X_BLUE = 70;
    public static int TOP_LEFT_Y_BLUE = 160;
    public static int BOTTOM_RIGHT_X_BLUE = 250;
    public static int BOTTOM_RIGHT_Y_BLUE = 230;

    public static double NORMALIZE_ALPHA = 51.0;
    public static double NORMALIZE_BETA = 261.0;

    public static double BLUR_RADIUS = 8.558558558558557;

    public static double HSV_THRESHOLD_HUE_MIN = 0.4668065215846204;
    public static double HSV_THRESHOLD_HUE_MAX = 1000;
    public static double HSV_THRESHOLD_SATURATION_MIN = 64;
    public static double HSV_THRESHOLD_SATURATION_MAX = 255.0;
    public static double HSV_THRESHOLD_VALUE_MIN = 109.84730100784292;
    public static double HSV_THRESHOLD_VALUE_MAX = 255.0;

    public static double MIN_CONTOUR_AREA = .1;

    public static double MIN_BLOB_SIZE = 4000;
    public static double VISION_ONE_TO_FOUR_ASPECT = 1.5;

    public static int WEBCAM_WIDTH = 320;
    public static int WEBCAM_HEIGHT = 240;

    //odometry positions all in meters
    public static double goalX = 0.9144;
    public static double goalY = 3.6576;
    public static double GOAL_RADIUS = 8;
    public static double POWER_SHOT_RADIUS = 1;
    public static double startingXOffset = 49/INCHES_PER_METER;
    public static double startingYOffset = ROBOT_RADIUS_INCHES/INCHES_PER_METER;
    public static double HEIGHT_MULTIPLIER = 1.35; //1.45 breeh
    public static double RPS_MULTIPLIER = 1.07;
    public static double MUZZLE_ANGLE_OFFSET_IN_TELE_OP = 2; //-5 breeh
    public static double STARTING_HEIGHT_OFFSET = 0;
    public static int  ELBOW_ZERO_DEGREES_OFFSET = 141;
    public static double ILLEGAL_SHOOTING_DISTANCE = 1.8288;

    public static int WOBBLE_GRIPPER_CLOSED = 2050;
    public static int WOBBLE_GRIPPER_CLOSED_2 = 2200;
    public static int WOBBLE_GRIPPER_RELEASE = 1400;
    public static int WOBBLE_GRIPPER_OPEN = 900;
    public static int GRIPPER_IN_POS = 0;
    public static int GRIPPER_OUT_POS = 550; //todo-fix this
    public static int GRIPPER_HALFWAY_POS = 550/2; //todo-fix this
    public static boolean IN_WOBBLE_MODE = false;
    //Dangerzone runs from turret at 8 degrees to 104 degrees relative to chassis
    public static double DANGER_ZONE_CENTER = 56;
    public static int DANGER_ZONE_WIDTH = 96;
    public static double DANGER_ZONE_SAFTEY_BUFFER = 3;
    public static int GRIPPER_HEADING_OFFSET = 42; //add this to the desired turret angle to actually point the open gripper that way and not the turret

    //renaming the Intake servo combinations to group together in Dashboard
    public static int INTAKE_DEPLOY_TOP = 1250; //1300
    public static int INTAKE_DEPLOY2_TOP = 1600;
    public static int INTAKE_DEPLOY_TRAVEL_BTM = 1600; //use as default bottom servo position if none specified

    public static int INTAKE_HANDOFF_BTM = 1760; //was 1675
    public static int INTAKE_HANDOFF_TOP = 1500; //was 1450
    public static int INTAKE_DEFLECTORANNOYING_TOP = 1350; //was 1400
    public static int INTAKE_HANDOFF__ROLLERS_TOP = 1200; //could be different if we are in active targeting and need to clear the slinger wall

    public static int INTAKE_INIT_BTM = 900;
    public static int INTAKE_INIT_TOP = 900;

    public static int INTAKE_PICKUP_TOP = 1680; //1500

    public static int INTAKE_TENT_BTM = 1500;//bruh.
    public static int INTAKE_TENT_TOP = 1750;
    public static int INTAKE_TENT_TOP2 = 1900; //do this after it has started moving forward

    public static int INTAKE_TRAVEL_TOP = 1450;

    public static double INTAKE_ROLLING_RING_NEAR = .15; //distance range to trigger snap back
    public static double INTAKE_ROLLING_RING_FAR = .42;
    public static double INTAKE_ROLLING_RING_TOO_FAR = .65;
    public static double INTAKE_ROLLING_RING_DELAY = .05;
    public static boolean INTAKE_ROLLING_JOG_FW_NOW = false; //request Pose to jog the robot backward briefly - helps separate touching rings
    public static double INTAKE_ROLLING_JOG_FW_TIME = .5;
    public static double INTAKE_ROLLING_JOG_FW_POWER = .8;
    public static boolean INTAKE_ROLLING_JOG_BW_NOW = false; //request Pose to jog the robot backward briefly - helps separate touching rings
    public static double INTAKE_ROLLING_JOG_BW_TIME = .5;
    public static double INTAKE_ROLLING_JOG_BW_POWER = -.8;
    public static int GRIPPER_INIT_POS = 125;
    public static int GRIPPER_TELEOP_INIT_POS = 100;

    //chassis-relative angle that places the transfer tray into a good position to receive rings from the intake
    public static int INTAKE_TO_TURRET_XFER_ANGLE = 0; //was 360-25 for feather gate
    public static int INTAKE_TO_TURRET_XFER_ELEVATION = 15;

    public static double INTAKE_SPEED = .8; //speed of walk and lift
    public static double INTAKE_TIME_FIRST = .65; //time to walk the ring
    public static double INTAKE_TIME_SECOND = 1.6; //time for ring to escalate & handoff
    public static boolean INTAKE_MINIJOG_NOW = false; //request Pose to jog the robot backward briefly - helps separate touching rings
    public static double INTAKE_MINIJOG_TIME = .35;
    public static double INTAKE_MINIJOG_POWER = .25;

    public static double INTAKE_AUTO_PICKUP_AMPS = 1.1;
    public static double INTAKE_AUTO_PICKUP_AMPS_LIM = .9;

    public static double TURRET_SPEED= 90; //max degrees per second to manually adjust turret targetAngle
    public static double TURRET_TOLERANCE = 2; //accuracy wiggle room
    public static double TURRET_OFFSET_HEADING = 0; // 5 breeh

    public static int LAUNCHER_TRIGGER_STOWED = 1900;
    public static int LAUNCHER_TRIGGER_SHOOT = 2040;
    public static int LAUNCHER_TRIGGER_BACK = 1780;
    public static int LAUNCHER_TRIGGER_FLIP = 1900;
    public static double autoLaunchTime = .1;
    public static double rampedUpPercent = 0.009;
    public static double INTAKE_AUTO_PICKUP_TIME_BUFFER = 1;

    public static int LAUNCHER_WIPER_UNWIPED = 2100;
    public static int LAUNCHER_WIPER_WIPED = 1500;

    public static double MIDFIELD_COLOR_RESET_POSITION = 3.5 * 12 / INCHES_PER_METER;

    public static double __ATMEP = 1;
    public static double __ATMEP2 = -1;

    //inner conflicts
    public static double TURRET_HEADING_OFFSET = 5;
    public static double ELBOW_LEGAL_ANGLE = 35;

    public static int WALL_FOLLOW_MULTIPLIER = 50;

    public enum Alliance {
        RED,
        BLUE;
    }

    public double getX(Target targ){
        return targ.x * ALLIANCE_INT_MOD;
}

    public enum Target { // 3 - 27 - 17.5
        NONE(0, 0, 0),
        HIGH_GOAL((12 * 3)/INCHES_PER_METER, (3 + 6 * 24) / INCHES_PER_METER, (3 * 12) / INCHES_PER_METER),
        MID_GOAL((12 * 3)/INCHES_PER_METER,(3 + 6 * 24) / INCHES_PER_METER,(27) / INCHES_PER_METER),
        MID_GOAL_CLASSIC((12 * -3)/INCHES_PER_METER,3.6576,(27) / INCHES_PER_METER),
        LOW_GOAL((12 * 3)/INCHES_PER_METER,(3 + 6 * 24) / INCHES_PER_METER,(17.5) / INCHES_PER_METER), //3.25 - 7.5
        FIRST_POWER_SHOT((3.5) / INCHES_PER_METER,(3 + 6 * 24) / INCHES_PER_METER,30 / INCHES_PER_METER),
        SECOND_POWER_SHOT((3.5 + 7.5) / INCHES_PER_METER,(3 + 6 * 24) / INCHES_PER_METER,30 / INCHES_PER_METER),
        THIRD_POWER_SHOT((3.5 + 2 * 7.5) / INCHES_PER_METER,(3 + 6 * 24) / INCHES_PER_METER,30 / INCHES_PER_METER),
        TEST_TARGET((12 * 3)/INCHES_PER_METER, (3 + 6 * 24) / INCHES_PER_METER, (3 * 12 + 8) / INCHES_PER_METER);

        public double getX(){
            return x  * ALLIANCE_INT_MOD;
        }

        public double x, y, height;

        private Target(double x, double y, double height) {
            this.x = x * ALLIANCE_INT_MOD;
            this.y = y;
            this.height = height;
        }

    }

    public enum Position {
        //headings and elevations that are negative means don't apply them to ending position - let other behaviors control
        START(49/INCHES_PER_METER, ROBOT_RADIUS_INCHES/INCHES_PER_METER,0,0,0, 1),
        START_INNER((49-24)/INCHES_PER_METER, ROBOT_RADIUS_INCHES/INCHES_PER_METER,0,0,0, 1),
        HOME(49/INCHES_PER_METER, (ROBOT_RADIUS_INCHES)/INCHES_PER_METER,0,0,0, 0),
        //WOBBLE_ONE_GRAB(48/INCHES_PER_METER, (ROBOT_RADIUS_INCHES+3)/INCHES_PER_METER,0,340,0),
        WOBBLE_ONE_GRAB(49/INCHES_PER_METER, (8 + ROBOT_RADIUS_INCHES)/INCHES_PER_METER,0,0,1,1),
        //340, 45
        ALIGNMENT_RESET(49/INCHES_PER_METER, 11*12/INCHES_PER_METER, 0,170,-1,1),
        //turret needs to rotate counter clockwise to deposit wobble goals A and C - use intermediate turret heading of 170
        //TARGET_C_1(49/INCHES_PER_METER, 10.5*12/INCHES_PER_METER, -1,190,-1,.2),
//        TARGET_C_1((49-3)/INCHES_PER_METER, 10.75*12/INCHES_PER_METER, -1,120,-1,.02),
//        TARGET_C_2((49)/INCHES_PER_METER, 9.75*12/INCHES_PER_METER, -1,45+45,-1,.02),
//        TARGET_C_3((49-12)/INCHES_PER_METER, 9.5*12/INCHES_PER_METER, -1,45+45,5,0),
        TARGET_B_1((49)/INCHES_PER_METER, 9*12/INCHES_PER_METER, 0,0,-1,.5), //        TARGET_B_1((49-7)/INCHES_PER_METER, 8.5*12/INCHES_PER_METER, 0,0,5,1),
        TARGET_B_2((49)/INCHES_PER_METER, (9.5*12 - 8)/INCHES_PER_METER, 0,0,-1,0.5),
        TARGET_A_1((49)/INCHES_PER_METER, (7*12 - 2)/INCHES_PER_METER, -1,90,20,.02),
        TARGET_A_1_BLUE((49)/INCHES_PER_METER, 7*12/INCHES_PER_METER, -1,0,20,.02),
        TARGET_A_2((49)/INCHES_PER_METER, (7*12 - 2)/INCHES_PER_METER, -1,90,-1,0),
        RING_STACK(36/INCHES_PER_METER, 48/INCHES_PER_METER,-1,-1, -1,0),
        RING_STACK_APPROACH(36/INCHES_PER_METER, (48+6+ ROBOT_RADIUS_INCHES)/INCHES_PER_METER, 180, 270,0,1), //sweep needs to be very slow
        RING_STACK_SWEEPTO(36/INCHES_PER_METER, (48-10+ ROBOT_RADIUS_INCHES)/INCHES_PER_METER, 180, 270,0,1),
        //this is the actual location of wobble2 for reference, not meant as a robot drive target
        WOBBLE_TWO(25/INCHES_PER_METER, 23/INCHES_PER_METER,-1,-1, -1,1),
        WOBBLE_TWO_APPROACH((49+3)/INCHES_PER_METER, (2.5 * 12)/INCHES_PER_METER, -1, -1,-1, 0),
        WOBBLE_TWO_APPROACH_BLUE((49+3)/INCHES_PER_METER, (2.5 * 12)/INCHES_PER_METER, 310, -1,-1, 0),
        WOBBLE_TWO_EXIT((49+3)/INCHES_PER_METER, (2.5*12)/INCHES_PER_METER, 350, 90 + GRIPPER_HEADING_OFFSET,15, 1),
        WOBBLE_TWO_EXIT_BLUE((49+3)/INCHES_PER_METER, (2.5*12)/INCHES_PER_METER, 350, 90 + GRIPPER_HEADING_OFFSET,15, 1),
        WOBBLE_TWO_EXIT_B((49+3)/INCHES_PER_METER, (2.5*12)/INCHES_PER_METER, 350, 0,-1, .02),
        WOBBLE_TWO_GRAB (30/INCHES_PER_METER, (30)/INCHES_PER_METER, -1, -1,-1,-1),
        NAVIGATE((49+6)/INCHES_PER_METER, 7*12/INCHES_PER_METER,-1,-1, -1, .2), //NAVIGATE(35/INCHES_PER_METER, 6.5*12/INCHES_PER_METER,-1,-1, -1, .5)
        NAVIGATE_INNER((49+6 - 30)/INCHES_PER_METER, 7*12/INCHES_PER_METER,-1,-1, -1, .2), //NAVIGATE(35/INCHES_PER_METER, 6.5*12/INCHES_PER_METER,-1,-1, -1, .5)
        LAUNCH_PREFERRED((49)/INCHES_PER_METER, (4.9*12)/INCHES_PER_METER,325,-1, -1,1), //LAUNCH_PREFERRED(3*12/INCHES_PER_METER, 5.5*12/INCHES_PER_METER,180,-1, -1,0)
        LAUNCH_PREFERRED_INNER((49 - 30)/INCHES_PER_METER, (4.9*12)/INCHES_PER_METER,315,-1, -1,1), //LAUNCH_PREFERRED(3*12/INCHES_PER_METER, 5.5*12/INCHES_PER_METER,180,-1, -1,0)
        TARGET_C_1((49)/INCHES_PER_METER, (8 * 12)/INCHES_PER_METER,-1,-1, -1,1),
        TARGET_C_2((49 - 24)/INCHES_PER_METER, (8 * 12)/INCHES_PER_METER,-1,-1, -1,1),
        LAUNCH_ROLLERS(39/INCHES_PER_METER, (70)/INCHES_PER_METER,260,-1, -1,.2), //LAUNCH_PREFERRED(3*12/INCHES_PER_METER, 5.5*12/INCHES_PER_METER,180,-1, -1,0)
        WOBBLE_GOAL_DUMP(49/INCHES_PER_METER, (ROBOT_RADIUS_INCHES + 3)/INCHES_PER_METER, -1, 180 + GRIPPER_HEADING_OFFSET, 45, .6),
        TEST_POS_FOR_TESTING(startingXOffset, startingYOffset+2,330,30, 10, .2);

        public double x, y, baseHeading, launchHeading, launchElevation, launchStart;

        public double getX(){return x * ALLIANCE_INT_MOD;}

        private Position(double x, double y, double baseHeading, double launchHeading, double launchElevation, double launchStart) {
            this.x = x;
            this.y = y;
            this.baseHeading=baseHeading; //-1 means take no action on changing
            this.launchHeading=launchHeading; //-1 means take no action on changing
            this.launchElevation=launchElevation; //-1 means take no action on changing
            this.launchStart=launchStart;  //number 0.0 to 1.0 - progression of travel where launcher actions should start
        }
    }
}