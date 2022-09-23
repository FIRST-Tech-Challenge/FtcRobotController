package org.firstinspires.ftc.teamcode.helpers;

//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 *
 */

public class Constants {

    // hardwareMap

    public static final String ELEVATOR_MOTOR = "elevator-motor";
    public static final String MOTOR_FRONT_RIGHT = "motor-front-right";
    public static final String MOTOR_FRONT_LEFT = "motor-front-left";
    public static final String MOTOR_BACK_RIGHT = "motor-back-right";
    public static final String MOTOR_BACK_LEFT = "motor-back-left";

    //private static final String WEIGHT_SENSOR_ZERO = "sensorAsAnalogInput0";
    //private static final String WEIGHT_SENSOR_ONE = "sensorAsAnalogInput1";
    //private static final String WEIGHT_SENSOR_TWO = "sensorAsAnalogInput2";
    //private static final String WEIGHT_SENSOR_THREE = "sensorAsAnalogInput3";
    //private static final String WEIGHT_LIGHT_RED = "weightIndicatorRed";
    //private static final String WEIGHT_LIGHT_GREEN = "weightIndicatorGreen";

    //private static final String WHEEL_MOTOR = "wheelMotor";

    public static final String IMU = "imu";
    public static final String INTAKE_MOTOR1 = "intakeLeft";
    public static final String INTAKE_MOTOR2 = "intakeRight";

    public static final String ROTATOR_MOTOR = "rotatorMotor";

    private static final String SPIN_WHEEL = "spin-wheel";
    public static final String WEBCAM = "Webcam 1";


    // Debugging captions
    public static final String ROBOT_SYSTEM_ERROR = "ROBOTSYSTEMERROR";

    private final static int CPS_STEP = 0x10000;

    public static final double TICKS_IN_MM = 3.51;

    public static final float mmPerInch = 25.4f;                    // constant for converting measurements from inches to millimeters
    public static final double MM_IN_TILE = mmPerInch * 24;
    public static final double TICKS_IN_TILE = 25.4 * MM_IN_TILE;

    public static int TICKS_PER_REV = 2400;
    public static int FULL_POWER_TILE_TIME = 2400; // TODO TEST THIS VALUE
    public static int TICKS_PER_REV_SHOOTER = 103;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 6.239; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0.60; // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.0;
    public static double Y_MULTIPLIER = 1.0;
    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double MAX_RPM = 200;

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static final PIDCoefficients MOTOR_VELO_PID = null;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */

    // TODO
    public static double TRACK_WIDTH = 30; // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
    /*public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            45, 30, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );*/


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MAX_RPM * TICKS_PER_REV);
    }

    //Camera
    public static final float CAMERA_FORWARD_DISPLACEMENT = 0 * mmPerInch;   // Position of the camera relative to the length
    public static final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;   // Position of the camera relative to the ground
    public static final float CAMERA_LEFT_DISPLACEMENT = 0;     // Position of the camera relative to the width

    //Field
    public static final double tileWidth = 23.5;

    //Vuforia
    public static final float mmTargetHeight = 6.25f * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;
    public static final float quadField = 36 * mmPerInch;

    //Shooter Constants
    public static final double powerShotY = tileWidth * 2.75;
    // TODO, Find position values.
    public static final double SHOOTING_SERVO_IDLE_POSITION = 0.5;
    public static final double SHOOTING_SERVO_SHOOT_POSITION = 0.3;

    //YeetSystem
    //TODO Change these numbers
    public static final double ARM_MOTOR_NUM_REVOLUTIONS = 0.66;
    public static final int ARM_MOTOR_DOWN_POSITION = (int)(Constants.TICKS_PER_REV * ARM_MOTOR_NUM_REVOLUTIONS);
    public static final int ARM_MOTOR_UP_POSITION = 0; // this needs to be changed
    public static final double ARM_MOTOR_RAW_POWER = 0.4;
    public static final double LEFT_ARM_SERVO_CLOSED_POSITION = 1.0;
    public static final double RIGHT_ARM_SERVO_CLOSED_POSITION = 0.45;
    public static final double LEFT_ARM_SERVO_OPEN_POSITION = 0.4;
    public static final double RIGHT_ARM_SERVO_OPEN_POSITION = 0.9;
    public static final int SERVO_WAIT_TIME = 350;
}
