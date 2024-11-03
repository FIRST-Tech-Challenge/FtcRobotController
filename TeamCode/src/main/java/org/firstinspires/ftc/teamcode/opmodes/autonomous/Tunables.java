package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.util.Units;

public class Tunables {
    /**
     * distance between left and right
     */
    public static final double TRACK_WIDTH = Units.inchesToMeters(15);
    /**
     * distance between front and rear
     */
    public static final double WHEEL_BASE = Units.inchesToMeters(12);
    /**
     * offset of center odometry from center
     */
    public static final double CENTER_WHEEL_OFFSET = Units.inchesToMeters(9);//Units.inchesToMeters(6.5);

    /**
     * max speed of robot for trajectory
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.05;

    /**
     * max acceleration of robot for trajectory
     */
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQ = 0.1;

    /**
     * Max rotation speed
     */
    public static final double MAX_ROTATION_SPEED = Math.PI/20; // 9 degree per second TODO: [tune]

    /**
     * Multiplier to convert angle to speed
     */
    public static final double ANGLE_TO_SPEED_MULTIPLIER = 0.5; // multiplier to convert angle to turn into speed TODO: [tune]

    /**
     * Rotation tolerance for trajectory
     */
    public static final double ROTATION_TOLERANCE = Math.toRadians(5); // 2 degree

    /**
     * x tolerance for trajectory
     */
    public static final double X_TOLERANCE = Units.inchesToMeters(0.2);

    /**
     * y tolerance for trajectory
     */
    public static final double Y_TOLERANCE = Units.inchesToMeters(0.2);

    public static final double ODOMETER_POD_WHEEL_DIAMETER_MM = 48;
    public static final double ODOMETER_POD_TICKS_PER_REVOLUTION = 2000;

    public static final double MECANUM_WHEEL_DIAMETER_MM = 90;
    public static final double MOTOR_ENCODER_TICKS_PER_REVOLUTION = 384.5;

    // PID related parameters
    public static final double X_KP = 1;
    public static final double X_KI = 0;
    public static final double X_KD = 0;

    public static final double Y_KP = 1;
    public static final double Y_KI = 0;
    public static final double Y_KD = 0;

    public static final double ROTATION_KP = 1;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    //Non trajectory auto parameters
    public static final double STOPPING_DISTANCE = 0.2; //20% of distance used to slow
    public static final double DRIVE_POWER = 0.5; //50% power when driving with no trajectory auto mode
    public static final double TOLERANCE = 0.001; //10mm tolerance
}
