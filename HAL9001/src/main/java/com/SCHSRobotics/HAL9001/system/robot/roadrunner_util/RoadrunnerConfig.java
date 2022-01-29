package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.DriveConfig;
import com.SCHSRobotics.HAL9001.util.math.units.HALAccelerationUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngularAccelerationUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngularVelocityUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALVelocityUnit;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.jetbrains.annotations.NotNull;

/**
 * A data class used to store roadrunner configuration options.
 * <p>
 * Creation Date: 1/5/21
 *
 * @author Roadrunner Quickstart; Cole Savage, Level Up
 * @version 1.0.0
 * @see DriveConfig
 * @since 1.1.1
 */
public class RoadrunnerConfig extends DriveConfig {

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public boolean USE_DRIVE_ENCODERS;
    public PIDFCoefficients MOTOR_VELO_PID;

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public double kV;
    public double kA;
    public double kStatic;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public double MAX_VEL;
    public double MAX_ACCEL;
    public double MAX_ANG_VEL;
    public double MAX_ANG_ACCEL;

    /*
     * Tolerance and timeout values for trajectory followers.
     */
    public double FOLLOWER_X_TOLERANCE = 0.5;
    public double FOLLOWER_Y_TOLERANCE = 0.5;
    public double FOLLOWER_HEADING_TOLERANCE = Math.toRadians(5.0);
    public double FOLLOWER_TIMEOUT = 0.5;

    /**
     * The constructor for RoadrunnerConfig.
     *
     * @param wheelRadius             The radius of the drivetrain's wheels.
     * @param wheelRadiusUnit         The unit for the wheelRadius parameter.
     * @param gearRatio               The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidth              The track width of the robot.
     * @param trackWidthUnit          The unit for the trackWidth parameter.
     * @param wheelBase               The wheel base of the robot.
     * @param wheelBaseUnit           The unit for the wheelBase parameter.
     * @param motorTicksPerRevolution The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM             The maximum RPM of the drivetrain motors.
     */
    public RoadrunnerConfig(double wheelRadius, HALDistanceUnit wheelRadiusUnit, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double wheelBase, HALDistanceUnit wheelBaseUnit, double motorTicksPerRevolution, double motorMaxRPM) {
        super(wheelRadius, wheelRadiusUnit, gearRatio, trackWidth, trackWidthUnit, wheelBase, wheelBaseUnit, motorTicksPerRevolution, motorMaxRPM);
        USE_DRIVE_ENCODERS = true;
        MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

        kV = 1.0 / rpmToVelocity(MAX_RPM);
        kA = 0;
        kStatic = 0;

        MAX_VEL = 30;
        MAX_ACCEL = 30;
        MAX_ANG_VEL = Math.toRadians(180);
        MAX_ANG_ACCEL = Math.toRadians(180);
    }

    /**
     * The constructor for RoadrunnerConfig.
     *
     * @param wheelRadiusInches The radius of the drivetrain's wheels in inches.
     * @param gearRatio         The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidthInches  The track width of the robot in inches.
     * @param wheelBaseInches   The wheel base of the robot in inches.
     * @param motorTicksPerRev  The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM       The maximum RPM of the drivetrain motors.
     */
    public RoadrunnerConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double wheelBaseInches, double motorTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidthInches, HALDistanceUnit.INCHES, wheelBaseInches, HALDistanceUnit.INCHES, motorTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for RoadrunnerConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadiusInches The radius of the drivetrain's wheels in inches.
     * @param gearRatio         The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidthInches  The track width of the robot in inches.
     * @param motorTicksPerRev  The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM       The maximum RPM of the drivetrain motors.
     */
    public RoadrunnerConfig(double wheelRadiusInches, double gearRatio, double trackWidthInches, double motorTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidthInches, HALDistanceUnit.INCHES, trackWidthInches, HALDistanceUnit.INCHES, motorTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for RoadrunnerConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadius        The radius of the drivetrain's wheels.
     * @param radiusUnit         The unit for the wheelRadius parameter.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public RoadrunnerConfig(double wheelRadius, HALDistanceUnit radiusUnit, double gearRatio, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadius, radiusUnit, gearRatio, 1, HALDistanceUnit.INCHES, 1, HALDistanceUnit.INCHES, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for RoadrunnerConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadiusInches  The radius of the drivetrain's wheels in inches.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public RoadrunnerConfig(double wheelRadiusInches, double gearRatio, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for RoadrunnerConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadius        The radius of the drivetrain's wheels.
     * @param radiusUnit         The unit for the wheelRadius parameter.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidth         The track width of the robot.
     * @param trackWidthUnit     The unit for the trackWidth parameter.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public RoadrunnerConfig(double wheelRadius, HALDistanceUnit radiusUnit, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadius, radiusUnit, gearRatio, trackWidth, trackWidthUnit, trackWidth, trackWidthUnit, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * The constructor for RoadrunnerConfig. Assumes a roughly square robot with a normal center of mass.
     *
     * @param wheelRadiusInches  The radius of the drivetrain's wheels in inches.
     * @param gearRatio          The gear ratio between the motors and the drivetrain wheels.
     * @param trackWidth         The track width of the robot.
     * @param trackWidthUnit     The unit for the trackWidth parameter.
     * @param encoderTicksPerRev The number of encoder ticks per revolution of the drivetrain motors.
     * @param motorMaxRPM        The maximum RPM of the drivetrain motors.
     */
    public RoadrunnerConfig(double wheelRadiusInches, double gearRatio, double trackWidth, HALDistanceUnit trackWidthUnit, double encoderTicksPerRev, double motorMaxRPM) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, trackWidth, trackWidthUnit, encoderTicksPerRev, motorMaxRPM);
    }

    /**
     * Calculates the motor velocity kF value given the motor's maximum number of ticks per second.
     * Calculation from <a href="https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx">this</a> document.
     *
     * @param ticksPerSecond The motor's maximum number of ticks per second
     * @return The motor's calculated velocity kF value.
     */
    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }

    /**
     * Whether or not the drivetrain is using drive encoders in its localizer.
     *
     * @param useDriveEncoders Whether or not the drivetrain is using drive encoders in its localizer.
     * @return This data object.
     */
    public RoadrunnerConfig useDriveEncoders(boolean useDriveEncoders) {
        USE_DRIVE_ENCODERS = useDriveEncoders;
        return this;
    }

    /**
     * Sets the motor velocity PID coefficients.
     *
     * @param motorVelocityPIDCoefficients The motor velocity PID coefficients.
     * @return This data object.
     */
    public RoadrunnerConfig setMotorVelocityPID(PIDFCoefficients motorVelocityPIDCoefficients) {
        MOTOR_VELO_PID = motorVelocityPIDCoefficients;
        return this;
    }

    /**
     * Sets the kV value used to model motor behavior.
     *
     * @param kV The kV value used to model motor behavior.
     * @return This data object.
     */
    public RoadrunnerConfig setKV(double kV) {
        this.kV = kV;
        return this;
    }

    /**
     * Sets the kA value used to model motor behavior.
     *
     * @param kA The kA value used to model motor behavior.
     * @return This data object.
     */
    public RoadrunnerConfig setKA(double kA) {
        this.kA = kA;
        return this;
    }

    /**
     * Sets the kStatic value used to model motor behavior.
     *
     * @param kStatic The kStatic value used to model motor behavior.
     * @return This data object.
     */
    public RoadrunnerConfig setKStatic(double kStatic) {
        this.kStatic = kStatic;
        return this;
    }

    /**
     * Sets the maximum velocity the drivetrain can have while following roadrunner trajectories.
     *
     * @param maxVelocity  The maximum velocity the drivetrain can have while following roadrunner trajectories.
     * @param velocityUnit The units of the maximum velocity.
     * @return This data object.
     */
    public RoadrunnerConfig setMaxVelocity(double maxVelocity, HALVelocityUnit velocityUnit) {
        MAX_VEL = HALVelocityUnit.convert(maxVelocity, velocityUnit, HALVelocityUnit.INCHES_PER_SECOND);
        return this;
    }

    /**
     * Sets the maximum acceleration the drivetrain can have while following roadrunner trajectories.
     *
     * @param maxAcceleration  The maximum acceleration the drivetrain can have while following roadrunner trajectories.
     * @param accelerationUnit The units of the maximum acceleration.
     * @return This data object.
     */
    public RoadrunnerConfig setMaxAcceleration(double maxAcceleration, HALAccelerationUnit accelerationUnit) {
        MAX_ACCEL = HALAccelerationUnit.convert(maxAcceleration, accelerationUnit, HALAccelerationUnit.INCHES_PER_SECOND_SQUARED);
        return this;
    }

    /**
     * Sets the maximum angular velocity the drivetrain can have while following roadrunner trajectories.
     *
     * @param maxAngularVelocity  The maximum angular velocity the drivetrain can have while following roadrunner trajectories.
     * @param angularVelocityUnit The units of the maximum angular velocity.
     * @return This data object.
     */
    public RoadrunnerConfig setMaxAngularVelocity(double maxAngularVelocity, HALAngularVelocityUnit angularVelocityUnit) {
        MAX_ANG_VEL = HALAngularVelocityUnit.convert(maxAngularVelocity, angularVelocityUnit, HALAngularVelocityUnit.RADIANS_PER_SECOND);
        return this;
    }

    /**
     * Sets the maximum angular acceleration the drivetrain can have while following roadrunner trajectories.
     *
     * @param maxAngularAcceleration  The maximum angular acceleration the drivetrain can have while following roadrunner trajectories.
     * @param angularAccelerationUnit The units of the maximum angular acceleration.
     * @return This data object.
     */
    public RoadrunnerConfig setMaxAngularAcceleration(double maxAngularAcceleration, HALAngularAccelerationUnit angularAccelerationUnit) {
        MAX_ANG_ACCEL = HALAngularAccelerationUnit.convert(maxAngularAcceleration, angularAccelerationUnit, HALAngularAccelerationUnit.RADIANS_PER_SECOND_SQUARED);
        return this;
    }

    /**
     * Sets the trajectory follower X position tolerance.
     *
     * @param xTolerance    The trajectory follower X position tolerance.
     * @param toleranceUnit The units of the trajectory follower's x position tolerance.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerXTolerance(double xTolerance, HALDistanceUnit toleranceUnit) {
        FOLLOWER_X_TOLERANCE = HALDistanceUnit.convert(xTolerance, toleranceUnit, HALDistanceUnit.INCHES);
        return this;
    }

    /**
     * Sets the trajectory follower X position tolerance.
     *
     * @param xToleranceInches The trajectory follower X position tolerance in inches.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerXTolerance(double xToleranceInches) {
        return setFollowerXTolerance(xToleranceInches, HALDistanceUnit.INCHES);
    }

    /**
     * Sets the trajectory follower Y position tolerance.
     *
     * @param yTolerance    The trajectory follower Y position tolerance.
     * @param toleranceUnit The units of the trajectory follower's Y position tolerance.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerYTolerance(double yTolerance, HALDistanceUnit toleranceUnit) {
        FOLLOWER_Y_TOLERANCE = HALDistanceUnit.convert(yTolerance, toleranceUnit, HALDistanceUnit.INCHES);
        return this;
    }

    /**
     * Sets the trajectory follower Y position tolerance.
     *
     * @param yToleranceInches The trajectory follower Y position tolerance in inches.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerYTolerance(double yToleranceInches) {
        return setFollowerYTolerance(yToleranceInches, HALDistanceUnit.INCHES);
    }

    /**
     * Sets the trajectory follower heading tolerance.
     *
     * @param headingTolerance The heading tolerance.
     * @param angleUnit        The units of the trajectory follower's heading tolerance.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerHeadingTolerance(double headingTolerance, @NotNull HALAngleUnit angleUnit) {
        FOLLOWER_HEADING_TOLERANCE = angleUnit.convertTo(HALAngleUnit.RADIANS).apply(headingTolerance);
        return this;
    }

    /**
     * Sets the trajectory follower heading tolerance.
     *
     * @param headingToleranceRadians The heading tolerance in radians.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerHeadingTolerance(double headingToleranceRadians) {
        return setFollowerHeadingTolerance(headingToleranceRadians, HALAngleUnit.RADIANS);
    }

    /**
     * Sets the trajectory follower's timeout value.
     *
     * @param timeout  The trajectory follower's timeout value.
     * @param timeUnit The units of the trajectory follower's timeout value.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerTimeout(double timeout, HALTimeUnit timeUnit) {
        FOLLOWER_TIMEOUT = HALTimeUnit.convert(timeout, timeUnit, HALTimeUnit.SECONDS);
        return this;
    }

    /**
     * Sets the trajectory follower's timeout value.
     *
     * @param timeoutSeconds The trajectory follower's timeout value in seconds.
     * @return This data object.
     */
    public RoadrunnerConfig setFollowerTimeout(double timeoutSeconds) {
        return setFollowerTimeout(timeoutSeconds, HALTimeUnit.SECONDS);
    }
}
