package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Math.Vector3;

@Config
public class Constants {
    // Drive Constants
    public static double DRIVE_FORWARD_POWER = 1;

    public static double X_CAM = -10, Y_CAM = 0, Z_CAM= -1.5;
    public static double YAW_CAM = Math.PI/2, PITCH_CAM = 0, ROLL_CAM = 0;
    public static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS,YAW_CAM, PITCH_CAM, ROLL_CAM, 0);//not used in new vision 725 class
    //tag info
    // tag ID 1 = left hive centered on box 3.25 inches above middle of the opening
    public static final double TAG_ID_1_Y_MAX = 1;
    public static final double TAG_ID_1_Y_MIN = -6;
    public static final double TAG_ID_1_Z_OFFSET = -3;

// extensions constants
    public static final double EXTENSION_MIN_POSITION = 0;
    public static final double EXTENSION_MAX_POSITION = 10;
    //shoulder constants
    public static final double SHOULDER_MAX_HEIGHT = 13;
    public static final double SHOULDER_MIN_HEIGHT = -18.5;
    //TODO: we should consider moving these values into the turret, arm and extension classes before building the redesign
    public static double TURRET_P =0.001, TURRET_I =0, TURRET_D = 0.00003;
    public static double ARM_P = 2.5, ARM_I = 0, ARM_D = .1;
    public static double ARM_ERROR = 0;
    public static double EXTENSION_P = 2.5, EXTENSION_I = 0, EXTENSION_D = .1;
    //the unit vector that points in the direction of the arm's home position 
    public Vector3 coordinateHome = new Vector3(1,0,0);

    // wrist constants
    public static final double WRIST_Z_OFFSET = .2; //5mm in inches this is the offset of the wrist when it is perpendicular to the hive assuming the ground is level
    public static final double WRIST_X_OFFSET = -7.337; //-186.3687mm in inches this is the offset of the wrist when it is perpendicular to the hive assuming the ground is level
    //turrent constants
    public static final Position TURRENT_TO_CAMERA = new Position (DistanceUnit.INCH,-8,4.5,7.5,0);
    public static final Position TURRENT_OFFSET_45_X_Y = new Position(DistanceUnit.INCH,-2,6.5,0,0);
    public static final Position TURRENT_OFFSET_90_X_Y = new Position(DistanceUnit.INCH,-8,11.5,0,0);
    public static final Position TURRENT_OFFSET_135_X_Y = new Position(DistanceUnit.INCH,-16,7.5,0,0);
    public static final Position TURRENT_OFFSET_180_X_Y = new Position(DistanceUnit.INCH,-20,2.5,0,0);
    public static final Position TURRENT_OFFSET_225_X_Y = new Position(DistanceUnit.INCH,-18,-6.5,0,0);

    public static final double ZERO_DEGREES = .589;
    public static final double FORTYFIVE_DEGREES = .873;
    public static final double NINETY_DEGREES = .997;
    public static final double ONETHIRTYFIVE_DEGREES = 1.252;
    public static final double ONEEIGHTY_DEGREES = 1.466;
    public static final double TWOTWENTYFIVE_DEGREES = 1.893;

    // turret min/max  analog potentiometer values
    public static final double TURRET_MIN_POSITION = .5; //1.26
    public static final double TURRET_MAX_POSITION = 2.45;

    public static final double TURRET_ERROR = .001;
}
