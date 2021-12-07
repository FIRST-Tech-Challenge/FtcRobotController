package org.firstinspires.ftc.teamcode.robots.reach.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRobotics40HdHexMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@Config
public class Constants {
    //misc

    public static final double INCHES_PER_METER = 39.3700787;
    public static Alliance ALLIANCE = Alliance.RED;
    public static int ALLIANCE_INT_MOD = 1;
    public static int numGameStates = 10;

    public static double _t = 0.0;
    public static double _q = 0.0;
    public static double _n = 0.0;
    public static double _e = 0.0;
    public static double _m = 0.0;

    //robot constants
    public static double swerveAngleTicksPerDegree = 0;
    public static double maxForwardSpeed = 1.2;
    public static double maxRotateSpeed = 1.2;
    public static double radiusOfDiff = 4; //todo- move to constants
    public static double wheelRadius = .9; //todo- make this a const. needs to be in meters
    public static double maxRotaryVelOfBackWheel = .9; //todo- calc this based on gear ratio
    public static double maxRotaryVelOfDiffWheels = .9; //todo- calc this based on gear ratio
    public static double TRACK_WIDTH = 0.308162;
    public static double CHASSIS_LENGTH = 1.0;
    public static double MOVEMENT_MULTIPLIER = 0.25;
    public static double DRIVETRAIN_TICKS_PER_REVOLUTION = MotorConfigurationType.getMotorType(RevRobotics40HdHexMotor.class).getTicksPerRev();

    public static double distanceP = 0.01;
    public static double distanceI = 0;
    public static double distanceD = 0;

    public static double TARGET_DISTANCE = 0.57;

    public static double startingXOffset = 0;
    public static double startingYOffset = 0;

    //vision
    public static int visionView = 0;
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


    public enum Alliance {
        RED,
        BLUE;
    }

    public enum Position {
        //headings and elevations that are negative means don't apply them to ending position - let other behaviors control
        START(49/INCHES_PER_METER, 49/INCHES_PER_METER,0, 0),
        START_INNER((49-24)/INCHES_PER_METER, 49/INCHES_PER_METER,0, 0),
        HOME(0, 0,0, 0);


        public double x, y, baseHeading, launchHeading, launchElevation, launchStart;

        public double getX(){return x * ALLIANCE_INT_MOD;}

        private Position(double x, double y, double baseHeading, double launchStart) {
            this.x = x;
            this.y = y;
            this.baseHeading=baseHeading; //-1 means take no action on changing
            this.launchStart=launchStart;  //number 0.0 to 1.0 - progression of travel where launcher actions should start
        }
    }
}