package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import static java.lang.Math.hypot;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * The base class for all HAL non-holonomic (non-omni-directional) Drivetrains
 * <p>
 * Creation Date: 1/10/21
 *
 * @author Cole Savage, Level Up.
 * @version 1.0.0
 * @see Drivetrain
 * @see TankDriveSimple
 * @see TankDrive
 * @since 1.1.1
 */
public abstract class NonHolonomicDrivetrain extends Drivetrain {
    //The drivetrain's current driving mode.
    protected DriveMode driveMode = DriveMode.STANDARD;
    //A weight that is applied to the drivetrain's velocity.
    protected double V_WEIGHT = 1;
    //The coefficients for the drivetrain's axial PID controller.
    protected PIDCoefficients axialCoefficients = new PIDCoefficients(0, 0, 0);
    //The drivetrain's axial PID controller.
    protected PIDFController axialController = new PIDFController(axialCoefficients);
    //The coefficients for the drivetrain's cross track PID controller.
    protected PIDCoefficients crossTrackCoefficients = new PIDCoefficients(0, 0, 0);
    //The drivetrain's cross track PID controller.
    protected PIDFController crossTrackController = new PIDFController(crossTrackCoefficients);

    /**
     * The base constructor for all non-holonomic drivetrains.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param config      The config names of all the motors in the drivetrain.
     */
    public NonHolonomicDrivetrain(Robot robot, DriveConfig driveConfig, String... config) {
        super(robot, driveConfig, config);
    }

    /**
     * Modifies the drivetrain's velocity using a variety of constants and velocity scaling methods.
     * Can be overriden to allow for custom functionality.
     *
     * @param power The drivetrain's input velocity.
     * @return The drivetrain's modified functionality.
     */
    protected double modifyPower(double power) {
        power *= constantSpeedMultiplier * V_WEIGHT;
        Vector2D powerVector = new Vector2D(power, 0);
        velocityScaleMethod.scaleFunction.accept(powerVector);
        power = powerVector.getX() * currentSpeedMultiplier;

        return power;
    }

    /**
     * A function that causes the drivetrain to move at the given power WITHOUT modifying the power.
     *
     * @param power The power to move at.
     */
    protected abstract void movePowerInternal(double power);

    /**
     * Causes the drivetrain to move at the specified power (after being modified by modifyPower).
     *
     * @param power The power to move at.
     */
    public final void movePower(double power) {
        movePowerInternal(modifyPower(power));
    }

    /**
     * Causes the drivetrain to move for a specified amount of time.
     *
     * @param power    The power to move at.
     * @param duration How long to move for.
     * @param timeUnit The units of the duration parameter.
     */
    public final void moveTime(double power, long duration, HALTimeUnit timeUnit) {
        movePower(power);
        waitTime((long) HALTimeUnit.convert(duration, timeUnit, HALTimeUnit.MILLISECONDS), () -> localizer.update());
        stopAllMotors();
    }

    /**
     * Causes the drivetrain to move for a specified amount of time.
     *
     * @param power      The power to move at.
     * @param durationMs How long to move for in milliseconds.
     */
    public final void moveTime(double power, long durationMs) {
        moveTime(power, durationMs, HALTimeUnit.MILLISECONDS);
    }

    /**
     * Causes the drivetrain to move by a specific amount.
     *
     * @param distance     The desired distance to move.
     * @param distanceUnit The units of the displacement vector.
     * @param power        The power to move at.
     */
    public final void moveSimple(double distance, HALDistanceUnit distanceUnit, double power) {
        Pose2d initialPose = localizer.getPoseEstimate();
        double distanceInches = HALDistanceUnit.convert(distance, distanceUnit, HALDistanceUnit.INCHES);

        movePower(power);
        waitWhile(() -> {
            Pose2d currentPose = localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseEstimate());
            return hypot(currentPose.getX() - initialPose.getX(), currentPose.getY() - initialPose.getY()) < distanceInches;
        }, () -> localizer.update());

        stopAllMotors();
    }

    /**
     * Causes the drivetrain to move by a specific amount.
     *
     * @param distance The desired distance to move.
     * @param power    The power to move at.
     */
    public final void moveSimple(double distance, double power) {
        moveSimple(distance, HALDistanceUnit.INCHES, power);
    }

    /**
     * Sets the drivetrain's driving mode.
     *
     * @param driveMode The drivetrain's driving mode.
     */
    public final void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    /**
     * Sets the drivetrain's velocity weight.
     *
     * @param velocityWeight The drivetrain's velocity weight.
     */
    public final void setVelocityWeight(double velocityWeight) {
        V_WEIGHT = velocityWeight;
    }

    /**
     * Sets the coefficients for the axial PID controller.
     *
     * @param pidCoefficients The coefficients for the axial PID controller.
     */
    public final void setAxialPID(PIDCoefficients pidCoefficients) {
        axialCoefficients = pidCoefficients;
        axialController = new PIDFController(pidCoefficients);
    }

    /**
     * Sets the coefficients for the cross track PID controller.
     *
     * @param pidCoefficients The cross track PID controller.
     */
    public final void setCrossTrackPID(PIDCoefficients pidCoefficients) {
        crossTrackCoefficients = pidCoefficients;
        crossTrackController = new PIDFController(crossTrackCoefficients);
    }

    /**
     * The drivetrain's drive mode.
     */
    public enum DriveMode {
        //Standard
        STANDARD,
        //Disabled
        DISABLED
    }
}