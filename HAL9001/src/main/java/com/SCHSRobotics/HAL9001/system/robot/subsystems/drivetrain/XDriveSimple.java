package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import static java.lang.Math.sqrt;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;

/**
 * A simple HAL X-drive subsystem. Does not include roadrunner.
 * <p>
 * Creation Date: 1/13/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see MecanumDriveSimple
 * @see MecanumDrive
 * @see XDrive
 * @see HolonomicDrivetrain
 * @see Drivetrain
 * @since 1.1.1
 */
public class XDriveSimple extends MecanumDriveSimple {

    /**
     * The constructor for XDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param topLeft     The top left motor config name.
     * @param topRight    The top right motor config name.
     * @param botLeft     The bottom left motor config name.
     * @param botRight    The bottom right motor config name.
     * @param useConfig   Whether or not the drivetrain uses the HAL config system.
     */
    public XDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight, boolean useConfig) {
        super(robot, driveConfig, topLeft, topRight, botLeft, botRight, useConfig);
    }

    /**
     * The constructor for XDriveSimple.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param topLeft     The top left motor config name.
     * @param topRight    The top right motor config name.
     * @param botLeft     The bottom left motor config name.
     * @param botRight    The bottom right motor config name.
     */
    public XDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, driveConfig, topLeft, topRight, botLeft, botRight, true);
    }

    @Override
    public double getMotorEncoderPosition(String motorName) {
        return sqrt(2) * super.getMotorEncoderPosition(motorName);
    }

    @Override
    public double getMotorVelocity(String motorName, HALAngleUnit angleUnit) {
        return sqrt(2) * super.getMotorVelocity(motorName, angleUnit);
    }
}
