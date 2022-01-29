package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import static java.lang.Math.sqrt;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.RoadrunnerConfig;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;

/**
 * A built-in HAL X-drive subsystem. Has roadrunner compatibility.
 * <p>
 * Creation Date: 1/13/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see MecanumDriveSimple
 * @see MecanumDrive
 * @see XDriveSimple
 * @see HolonomicDrivetrain
 * @see Drivetrain
 * @see com.acmerobotics.roadrunner.drive.MecanumDrive
 * @since 1.1.1
 */
public class XDrive extends MecanumDrive {

    /**
     * The constructor for XDrive.
     *
     * @param robot     The robot using this drivetrain.
     * @param rrConfig  The roadrunner config settings and drivetrain hardware constraints.
     * @param topLeft   The top left motor config name.
     * @param topRight  The top right motor config name.
     * @param botLeft   The bottom left motor config name.
     * @param botRight  The bottom right motor config name.
     * @param useConfig Whether or not the drivetrain uses the HAL config system.
     */
    public XDrive(Robot robot, RoadrunnerConfig rrConfig, String topLeft, String topRight, String botLeft, String botRight, boolean useConfig) {
        super(robot, rrConfig, topLeft, topRight, botLeft, botRight, useConfig);
    }

    /**
     * The constructor for XDrive.
     *
     * @param robot    The robot using this drivetrain.
     * @param rrConfig The roadrunner config settings and drivetrain hardware constraints.
     * @param topLeft  The top left motor config name.
     * @param topRight The top right motor config name.
     * @param botLeft  The bottom left motor config name.
     * @param botRight The bottom right motor config name.
     */
    public XDrive(Robot robot, RoadrunnerConfig rrConfig, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, rrConfig, topLeft, topRight, botLeft, botRight, true);
    }

    @Override
    public double getMotorEncoderPosition(String motorName) {
        return sqrt(2) * super.getMotorEncoderPosition(motorName);
    }

    @Override
    public double getMotorVelocity(String motorName, HALAngleUnit angleUnit) {
        return super.getMotorVelocity(motorName, angleUnit);
    }
}
