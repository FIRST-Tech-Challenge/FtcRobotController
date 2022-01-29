package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;

/**
 * A data class used for storing important information about the robot's hardware constraints.
 * <p>
 * Creation date: 1/7/21
 *
 * @author Raodrunner Quickstart; Cole Savage, Level Up.
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.RoadrunnerConfig
 * @see com.SCHSRobotics.HAL9001.system.robot.localizer.TrackingWheelConfig
 * @since 1.1.1
 */
public class DriveConfig {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    public final double TICKS_PER_REV;
    public final double MAX_RPM;

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public final double WHEEL_RADIUS; // in
    public final double GEAR_RATIO; // output (wheel) speed / input (motor) speed
    public final double TRACK_WIDTH; // in
    public final double WHEEL_BASE; //in

    /**
     * The constructor for DriveConfig.
     *
     * @param wheelRadius      The radius of the drivetrain's wheels.
     * @param wheelRadiusUnit  The unit for the wheelRadius parameter.
     * @param gearRatio        The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidth       The track width of the robot.
     * @param trackWidthUnit   The unit for the trackWidth parameter.
     * @param wheelBase        The wheel base of the robot.
     * @param wheelBaseUnit    The unit for the wheelBase parameter.
     * @param motorTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM      The maximum RPM of the drivetrain motors.
     */
    public DriveConfig(double wheelRadius, HALDistanceUnit wheelRadiusUnit, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double wheelBase, HALDistanceUnit wheelBaseUnit, double motorTicksPerRev, double motorMaxRPM) {
        WHEEL_RADIUS = HALDistanceUnit.convert(wheelRadius, wheelRadiusUnit, HALDistanceUnit.INCHES);
        GEAR_RATIO = gearRatio;
        TRACK_WIDTH = HALDistanceUnit.convert(trackWidth, trackWidthUnit, HALDistanceUnit.INCHES);
        WHEEL_BASE = HALDistanceUnit.convert(wheelBase, wheelBaseUnit, HALDistanceUnit.INCHES);
        TICKS_PER_REV = motorTicksPerRev;
        MAX_RPM = motorMaxRPM;
    }

    /**
     * The constructor for DriveConfig.
     *
     * @param wheelRadiusInches The radius of the drivetrain's wheels in inches.
     * @param gearRatio         The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidthInches  The track width of the robot in inches.
     * @param wheelBaseInches   The wheel base of the robot in inches.
     * @param motorTicksPerRev  The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM       The maximum RPM of the drivetrain motors.
     */
    public DriveConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double wheelBaseInches, double motorTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidthInches, HALDistanceUnit.INCHES, wheelBaseInches, HALDistanceUnit.INCHES, motorTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for DriveConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadiusInches The radius of the drivetrain's wheels in inches.
     * @param gearRatio         The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidthInches  The track width of the robot in inches.
     * @param motorTicksPerRev  The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM       The maximum RPM of the drivetrain motors.
     */
    public DriveConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double motorTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidthInches, HALDistanceUnit.INCHES, trackWidthInches, HALDistanceUnit.INCHES, motorTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for DriveConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadius        The radius of the drivetrain's wheels.
     * @param radiusUnit         The unit for the wheelRadius parameter.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public DriveConfig(double wheelRadius, HALDistanceUnit radiusUnit, double gearRatio, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadius, radiusUnit, gearRatio, 1, HALDistanceUnit.INCHES, 1, HALDistanceUnit.INCHES, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for DriveConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadiusInches  The radius of the drivetrain's wheels in inches.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public DriveConfig(double wheelRadiusInches, double gearRatio, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for DriveConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadius        The radius of the drivetrain's wheels.
     * @param radiusUnit         The unit for the wheelRadius parameter.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidth         The track width of the robot.
     * @param trackWidthUnit     The unit for the trackWidth parameter.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public DriveConfig(double wheelRadius, HALDistanceUnit radiusUnit, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadius, radiusUnit, gearRatio, trackWidth, trackWidthUnit, trackWidth, trackWidthUnit, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for DriveConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadiusInches  The radius of the drivetrain's wheels in inches.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidth         The track width of the robot.
     * @param trackWidthUnit     The unit for the trackWidth parameter.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public DriveConfig(double wheelRadiusInches, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidth, trackWidthUnit, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * Converts encoder ticks to inches using driveconfig's inputted data.
     *
     * @param ticks The number of encoder ticks travelled.
     * @return The number of inches travelled.
     */
    public final double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /**
     * Converts the rpm of a drivetrain motor to the velocity of a drivetrain wheel.
     *
     * @param rpm The rpm of a drivetrain motor.
     * @return The velocity of a connected drivetrain wheel.
     */
    public final double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
}
