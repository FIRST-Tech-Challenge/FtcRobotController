package com.SCHSRobotics.HAL9001.system.robot.localizer;

import com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain.DriveConfig;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;

/**
 * A data class used for storing hardware constraints for tracking wheels.
 * <p>
 * Creation Date: 1/8/21
 *
 * @author Cole Savage, Level UP
 * @version 1.0.0
 * @see DriveConfig
 * @since 1.1.1
 */
public class TrackingWheelConfig extends DriveConfig {

    /**
     * The constructor for TrackingWheelConfig.
     *
     * @param wheelRadius        The radius of the tracking wheels.
     * @param radiusUnit         The unit for the wheelRadius parameter.
     * @param gearRatio          The gear ratio between the tracking wheels and encoder.
     * @param encoderTicksPerRev The number of encoder ticks in one revolution.
     */
    public TrackingWheelConfig(double wheelRadius, HALDistanceUnit radiusUnit, double gearRatio, double encoderTicksPerRev) {
        super(wheelRadius, radiusUnit, gearRatio, 1, HALDistanceUnit.INCHES, 1, HALDistanceUnit.INCHES, encoderTicksPerRev, 1);
    }

    /**
     * The constructor for TrackingWheelConfig.
     *
     * @param wheelRadiusInches  The radius of the tracking wheels in inches.
     * @param gearRatio          The gear ratio between the tracking wheels and encoder.
     * @param encoderTicksPerRev The number of encoder ticks in one revolution.
     */
    public TrackingWheelConfig(double wheelRadiusInches, double gearRatio, double encoderTicksPerRev) {
        this(wheelRadiusInches, HALDistanceUnit.INCHES, gearRatio, encoderTicksPerRev);
    }
}
