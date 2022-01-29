package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import static java.lang.Math.abs;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALDistanceUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;

/**
 * The base class for all HAL holonomic (omni-directional) Drivetrains
 * <p>
 * Creation Date: 1/5/21
 *
 * @author Cole Savage, Level Up.
 * @version 1.0.0
 * @see Drivetrain
 * @see MecanumDriveSimple
 * @see MecanumDrive
 * @see XDriveSimple
 * @see XDrive
 * @since 1.1.1
 */
public abstract class HolonomicDrivetrain extends Drivetrain {
    //Weights that are applied to the drivetrain x and y velocity components.
    protected double
            VX_WEIGHT = 1,
            VY_WEIGHT = 1;
    //A weight applied to the drivetrain's lateral movement.
    protected double LATERAL_MULTIPLIER = 1;

    //The PID coefficients for the translational PID controller.
    protected PIDCoefficients translationCoefficients = new PIDCoefficients(0, 0, 0);
    //The translational PID controller.
    protected PIDFController translationController = new PIDFController(translationCoefficients);
    //The drivetrain's current driving mode (STANDARD/robot centric or FIELD_CENTRIC).
    protected DriveMode driveMode = DriveMode.STANDARD;

    /**
     * The base constructor for all holonomic drivetrains.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param config      The config names of all the motors in the drivetrain.
     */
    public HolonomicDrivetrain(Robot robot, DriveConfig driveConfig, String... config) {
        super(robot, driveConfig, config);
    }

    /**
     * Modifies the drivetrain's velocity using a variety of constants and velocity scaling methods.
     * Can be overriden to allow for custom functionality.
     *
     * @param power The drivetrain's input velocity.
     * @return The drivetrain's modified functionality.
     */
    protected Vector2D modifyPower(@NotNull Vector2D power) {
        Vector2D transformedPowerVector = new Vector2D(power.getX() * VX_WEIGHT, power.getY() * VY_WEIGHT).multiply(constantSpeedMultiplier);
        velocityScaleMethod.scaleFunction.accept(transformedPowerVector);
        transformedPowerVector.multiply(currentSpeedMultiplier);
        if (transformedPowerVector.magnitude() > velocityCap) {
            transformedPowerVector.normalize().multiply(velocityCap);
        }

        return transformedPowerVector;
    }

    /**
     * A function that causes the drivetrain to move at the given power WITHOUT modifying the power.
     *
     * @param power The power to move at.
     */
    protected abstract void movePowerInternal(Vector2D power);

    public final void movePower(double xPower, double yPower) {
        Vector2D power = coordinateMode.convertVectorTo(CoordinateMode.HAL).apply(new Vector2D(xPower, yPower));
        if (driveMode == DriveMode.FIELD_CENTRIC) {
            power.rotate(-localizer.getPoseEstimate().getHeading());
        }
        movePowerInternal(modifyPower(power));
    }

    /**
     * Causes the drivetrain to move at the specified power (after being modified by modifyPower).
     *
     * @param power The power to move at.
     */
    public final void movePower(Vector2D power) {
        movePower(power.getX(), power.getY());
    }

    public final void movePower(double power, double angle, HALAngleUnit angleUnit) {
        Vector2D inputVector = new Vector2D(power, angle, angleUnit);
        //90 degree ccw rotation.
        movePower(-inputVector.getY(), inputVector.getX());
    }

    public final void moveTime(double xPower, double yPower, long duration, HALTimeUnit timeUnit) {
        movePower(xPower, yPower);
        waitTime((long) HALTimeUnit.convert(duration, timeUnit, HALTimeUnit.MILLISECONDS), () -> localizer.update());
        stopAllMotors();
    }

    public final void moveTime(double xPower, double yPower, long durationMs) {
        moveTime(xPower, yPower, durationMs, HALTimeUnit.MILLISECONDS);
    }

    public final void moveTime(double power, double angle, HALAngleUnit angleUnit, long duration, HALTimeUnit timeUnit) {
        Vector2D inputVector = new Vector2D(power, angle, angleUnit);
        //90 degree ccw rotation.
        moveTime(-inputVector.getY(), inputVector.getX(), duration, timeUnit);
    }

    public final void moveTime(double power, double angle, HALAngleUnit angleUnit, long durationMs) {
        moveTime(power, angle, angleUnit, durationMs, HALTimeUnit.MILLISECONDS);
    }

    /**
     * Causes the drivetrain to move for a specified amount of time.
     *
     * @param power    The power to move at.
     * @param duration How long to move for.
     * @param timeUnit The units of the duration parameter.
     */
    public final void moveTime(Vector2D power, long duration, HALTimeUnit timeUnit) {
        moveTime(power.getX(), power.getY(), duration, timeUnit);
    }

    /**
     * Causes the drivetrain to move for a specified amount of time.
     *
     * @param power      The power to move at.
     * @param durationMs How long to move for in milliseconds.
     */
    public final void moveTime(Vector2D power, long durationMs) {
        moveTime(power, durationMs, HALTimeUnit.MILLISECONDS);
    }

    public final void moveSimple(double xDisplacement, double yDisplacement, HALDistanceUnit distanceUnit, double power) {
        Pose2d initialPose = localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseEstimate());

        //Convert from given distance unit to inches.
        Vector2D displacement = coordinateMode.convertVectorTo(CoordinateMode.HAL).apply(
                new Vector2D(
                        HALDistanceUnit.convert(xDisplacement, distanceUnit, HALDistanceUnit.INCHES),
                        HALDistanceUnit.convert(yDisplacement, distanceUnit, HALDistanceUnit.INCHES)
                )
        );

        Vector2D velocity = displacement.copy().normalize().multiply(Range.clip(power, -1, 1));

        //Change the displacement vector to robot-centric mode if needed.
        if (driveMode != DriveMode.FIELD_CENTRIC) {
            displacement.rotate(localizer.getPoseEstimate().getHeading());
        }

        movePower(velocity);
        waitWhile(() -> {
            Pose2d currentPose = localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseEstimate());
            //System.out.println(currentPose);
            return abs(currentPose.getX() - initialPose.getX()) < abs(displacement.getX()) || abs(currentPose.getY() - initialPose.getY()) < abs(displacement.getY());
        }, () -> localizer.update());

        stopAllMotors();
    }

    public final void moveSimple(double xDisplacement, double yDisplacement, double power) {
        moveSimple(xDisplacement, yDisplacement, HALDistanceUnit.INCHES, power);
    }

    public final void moveSimple(double displacement, HALDistanceUnit distanceUnit, double angle, HALAngleUnit angleUnit, double power) {
        Vector2D inputVector = new Vector2D(displacement, angle, angleUnit);
        //90 degree ccw rotation.
        moveSimple(-inputVector.getY(), inputVector.getX(), distanceUnit, power);
    }

    public final void moveSimple(double displacementInches, double angle, HALAngleUnit angleUnit, double power) {
        moveSimple(displacementInches, HALDistanceUnit.INCHES, angle, angleUnit, power);
    }

    /**
     * Causes the drivetrain to move by a specific amount. Note: This is affected by field centric vs robot-centric coordinates and HAL vs roadrunner coordinates.
     *
     * @param displacement The drivetrain's desired displacement vector.
     * @param distanceUnit The units of the displacement vector.
     * @param power        The power to move at.
     */
    public final void moveSimple(@NotNull Vector2D displacement, HALDistanceUnit distanceUnit, double power) {
        moveSimple(displacement.getX(), displacement.getY(), distanceUnit, power);
    }

    /**
     * Causes the drivetrain to move by a specific amount. Note: This is affected by field centric vs robot-centric coordinates and HAL vs roadrunner coordinates.
     *
     * @param displacement The drivetrain's desired displacement vector (units are in inches).
     * @param power        The power to move at.
     */
    public final void moveSimple(Vector2D displacement, double power) {
        moveSimple(displacement, HALDistanceUnit.INCHES, power);
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
     * Sets the coefficients for the translational PID controller.
     *
     * @param pidCoefficients The coefficients for the translational PID controller.
     */
    public final void setTranslationalPID(PIDCoefficients pidCoefficients) {
        translationCoefficients = pidCoefficients;
        translationController = new PIDFController(translationCoefficients);
    }

    /**
     * Gets the velocity X weight.
     *
     * @return The velocity X weight.
     */
    public final double getVelocityXWeight() {
        return VX_WEIGHT;
    }

    /**
     * Sets the velocity X weight.
     *
     * @param velocityXWeight The velocity X weight.
     */
    public final void setVelocityXWeight(double velocityXWeight) {
        VX_WEIGHT = velocityXWeight;
    }

    /**
     * Gets the velocity Y weights.
     *
     * @return The velocity Y weight.
     */
    public final double getVelocityYWeight() {
        return VY_WEIGHT;
    }

    /**
     * Sets the velocity Y weight.
     *
     * @param velocityYWeight The velocity Y weight.
     */
    public final void setVelocityYWeight(double velocityYWeight) {
        VY_WEIGHT = velocityYWeight;
    }

    /**
     * Gets the velocity lateral multiplier.
     *
     * @return The velocity lateral multiplier.
     */
    public final double getLateralMultiplier() {
        return LATERAL_MULTIPLIER;
    }

    /**
     * Sets the drivetrain's lateral multiplier.
     *
     * @param lateralMultiplier The drivetrain's lateral multiplier.
     */
    public final void setLateralMultiplier(double lateralMultiplier) {
        LATERAL_MULTIPLIER = lateralMultiplier;
    }

    /**
     * The drivetrain's driving mode.
     */
    public enum DriveMode {
        //Robot centric driving.
        STANDARD,
        //Field centric driving.
        FIELD_CENTRIC,
        //Drivetrain disabled.
        DISABLED
    }
}
