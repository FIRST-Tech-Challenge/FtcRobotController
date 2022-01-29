package com.SCHSRobotics.HAL9001.system.robot.subsystems.drivetrain;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.signum;

import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.system.robot.roadrunner_util.CoordinateMode;
import com.SCHSRobotics.HAL9001.util.math.geometry.Vector2D;
import com.SCHSRobotics.HAL9001.util.math.units.HALAngleUnit;
import com.SCHSRobotics.HAL9001.util.math.units.HALTimeUnit;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;

/**
 * The base class for all HAL Drivetrains
 * <p>
 * Creation Date: 1/5/21
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see HolonomicDrivetrain
 * @see NonHolonomicDrivetrain
 * @since 1.1.1
 */
public abstract class Drivetrain extends SubSystem {
    //The configuration settings for the drivetrain. Describes hardware constraints.
    public final DriveConfig driveConfig;
    //A hashmap linking motor names to motor objects.
    protected Map<String, DcMotorEx> motors = new HashMap<>();
    //The battery voltage sensor.
    protected VoltageSensor batteryVoltageSensor;
    //The localizer used to determine the position of the robot on the field.
    protected Localizer localizer;
    //The coordinate mode used to enter commands and the coordinate mode of the localizer.
    protected CoordinateMode coordinateMode, localizerCoordinateMode;
    //The PID coefficients for the heading PID and the turn-to-angle PID.
    protected PIDCoefficients
            headingCoefficients = new PIDCoefficients(0, 0, 0),
            turnCoefficients = new PIDCoefficients(0, 0, 0);
    //The heading PID controller and the turn-to-angle PID controller.
    protected PIDFController
            headingController = new PIDFController(headingCoefficients),
            turnController = new PIDFController(turnCoefficients);
    //The method used to scale the turning velocity and normal velocity of the drivetrain.
    protected SpeedScaleMethod
            velocityScaleMethod = SpeedScaleMethod.NONE,
            turnSpeedScaleMethod = SpeedScaleMethod.NONE;
    //A set of constants that are used to modify the drivetrain's speed and turning speed.
    protected double
            constantSpeedMultiplier = 1,
            constantTurnSpeedMultiplier = 1,
            currentSpeedMultiplier = 1,
            currentTurnSpeedMultiplier = 1,
            velocityCap = 1,
            turnSpeedCap = 1;
    /*A constant used to weight the robot's angular velocity.
     (functionally the same as constantTurnSpeedMultiplier, but added for roadrunner compatibility).*/
    protected double OMEGA_WEIGHT = 1;
    //The tolerance value for the turn-to-angle PID.
    protected double headingAngleToleranceRadians = 1e-2;

    /**
     * The base constructor for HAL Drivetrains.
     *
     * @param robot       The robot using this drivetrain.
     * @param driveConfig The driveconfig, which gives basic hardware constraints of the drivetrain.
     * @param config      The config names of all the motors in the drivetrain.
     */
    public Drivetrain(Robot robot, DriveConfig driveConfig, @NotNull String... config) {
        super(robot);

        this.driveConfig = driveConfig;

        for (String motorName : config) {
            motors.put(motorName, robot.hardwareMap.get(DcMotorEx.class, motorName));
        }
        resetMotorEncoders();
        setAllMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);
        setAllMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        batteryVoltageSensor = robot.hardwareMap.voltageSensor.iterator().next();

        turnController.setInputBounds(-PI, PI);
        headingController.setInputBounds(-PI, PI);

        headingController.setTargetPosition(0);

        coordinateMode = CoordinateMode.HAL;
        localizerCoordinateMode = CoordinateMode.HAL;
    }

    /**
     * Gets an array of all the motor config names.
     *
     * @return An array of all the motor config names.
     */
    @NotNull
    public final String[] getMotorConfig() {
        return motors.keySet().toArray(new String[0]);
    }

    /**
     * Gets an array of all the drivetrain motors.
     *
     * @return An array of all the drivetrain motors.
     */
    @NotNull
    public final DcMotorEx[] getMotors() {
        return motors.values().toArray(new DcMotorEx[0]);
    }

    /**
     * Gets a specific motor object by its config name.
     *
     * @param motorName The name of the motor to return.
     * @return The motor associated with that name.
     */
    public final DcMotorEx getMotor(String motorName) {
        return motors.get(motorName);
    }

    /**
     * Resets all drivetrain motor encoders to 0.
     */
    public final void resetMotorEncoders() {
        for (DcMotorEx motor : motors.values()) {
            DcMotor.RunMode runMode = motor.getMode();
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(runMode);
        }
    }

    /**
     * Stops all drivetrain motors.
     */
    public final void stopAllMotors() {
        for (DcMotorEx motor : motors.values()) {
            motor.setPower(0);
        }
    }

    /**
     * Sets the zero power behavior of all drivetrain motors.
     *
     * @param zeroPowerBehavior The zero power behavior of all drivetrain motors.
     */
    public final void setAllMotorZeroPowerBehaviors(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors.values()) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Sets the runmode of all drivetrain motors.
     *
     * @param runMode The runmode of all drivetrain motors.
     */
    public final void setAllMotorModes(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors.values()) {
            motor.setMode(runMode);
        }
    }

    /**
     * Sets the motor PIDF coefficients and runmode for all drivetrain motors.
     *
     * @param runMode          The runmode of the motor.
     * @param pidfCoefficients The motor's PIDF coefficients.
     */
    public final void setMotorPIDFCoefficients(DcMotor.RunMode runMode, @NotNull PIDFCoefficients pidfCoefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d,
                pidfCoefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors.values()) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    /**
     * Sets the power of a specific motor.
     *
     * @param motorName  The name of the motor.
     * @param motorPower The power to set the motor to run at.
     */
    public final void setMotorPower(String motorName, double motorPower) {
        getMotor(motorName).setPower(motorPower);
    }

    /**
     * Sets the runmode of a specific motor.
     *
     * @param motorName The name of the motor.
     * @param mode      The mode to set the motor to.
     */
    public final void setMotorMode(String motorName, DcMotor.RunMode mode) {
        getMotor(motorName).setMode(mode);
    }

    /**
     * Sets the zero power behavior of a specific motor.
     *
     * @param motorName         The name of the motor.
     * @param zeroPowerBehavior The zero power behavior to set the motor to.
     */
    public final void setMotorZeroPowerBehavior(String motorName, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        getMotor(motorName).setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Reverses a specific motor.
     *
     * @param motorName The name of the motor.
     */
    public final void reverseMotor(String motorName) {
        DcMotorEx motor = getMotor(motorName);
        motor.setDirection(motor.getDirection() == DcMotorSimple.Direction.FORWARD ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Sets the direction of a specific motor.
     *
     * @param motorName The name of the motor.
     * @param direction The direction to set the motor to.
     */
    public final void setMotorDirection(String motorName, DcMotorSimple.Direction direction) {
        getMotor(motorName).setDirection(direction);
    }

    /**
     * Gets the motor encoder positon of a specific motor. Note that this can be overriden in extended classes if needed.
     *
     * @param motorName The name of the motor.
     * @return That motor's encoder position.
     */
    public double getMotorEncoderPosition(String motorName) {
        return getMotor(motorName).getCurrentPosition();
    }

    /**
     * Gets the velocity of a specific motor. Note that this can be overriden in extended classes if needed.
     *
     * @param motorName The name of the motor.
     * @param angleUnit The angle unit of the motor velocity.
     * @return That motor's velocity.
     */
    public double getMotorVelocity(String motorName, HALAngleUnit angleUnit) {
        return getMotor(motorName).getVelocity(angleUnit == HALAngleUnit.RADIANS ? AngleUnit.RADIANS : AngleUnit.DEGREES);
    }

    /**
     * Gets the velocity of a specific motor. Note that this can be overriden in extended classes if needed.
     * The angle unit of the velocity defaults to radians.
     *
     * @param motorName The name of the motor.
     * @return That motor's velocity.
     */
    public double getMotorVelocity(String motorName) {
        return getMotor(motorName).getVelocity();
    }

    /**
     * Modifies the drivetrain's turning power using a variety of constants and velocity scaling methods.
     * Can be overriden to allow for custom functionality.
     *
     * @param turnPower The drivetrain's turning power.
     * @return A modified version of the drivetrain's turning power.
     */
    protected double modifyTurnPower(double turnPower) {
        turnPower *= constantTurnSpeedMultiplier * OMEGA_WEIGHT;
        Vector2D turnVector = new Vector2D(turnPower, 0);
        turnSpeedScaleMethod.scaleFunction.accept(turnVector);

        turnPower = turnVector.getX() * currentTurnSpeedMultiplier;
        turnPower = signum(turnPower) * min(abs(turnPower), turnSpeedCap);
        return turnPower;
    }

    /**
     * A function that causes the drivetrain to turn at the given power WITHOUT modifying the turn power.
     *
     * @param power The power to turn at.
     */
    protected abstract void turnPowerInternal(double power);

    /**
     * Causes the drivetrain to turn at the specified power (after being modified by modifyTurnPower).
     *
     * @param power The power to turn at.
     */
    public final void turnPower(double power) {
        power = modifyTurnPower(power);
        turnPowerInternal(power);
    }

    /**
     * Causes the drivetrain to turn for a certain period of time.
     *
     * @param power    The power to turn at.
     * @param duration How long to turn for.
     * @param timeUnit The units of the duration parameter.
     */
    public final void turnTime(double power, double duration, HALTimeUnit timeUnit) {
        turnPower(power);
        waitTime((long) HALTimeUnit.convert(duration, timeUnit, HALTimeUnit.MILLISECONDS), () -> localizer.update());
        stopAllMotors();
    }

    /**
     * Causes the drivetrain to turn for a certain period of time.
     *
     * @param power      The power to turn at.
     * @param durationMs How long to turn for in milliseconds.
     */
    public final void turnTime(double power, long durationMs) {
        turnTime(power, durationMs, HALTimeUnit.MILLISECONDS);
    }

    /**
     * Causes the drivetrain to turn by a specific amount.
     * This amount is usually an angle in radians, but if the localizer is not working correctly
     * or the driveconfig is incorrect, can be something else.
     * <p>
     * Counterclockwise is positive, clockwise is negative.
     *
     * @param power        The power to turn at.
     * @param angleRadians The angle to turn by in radians.
     */
    public final void turnSimple(double power, double angleRadians) {
        Pose2d initialPose = localizer.getPoseEstimate();

        if (power != 0) {
            turnPower(angleRadians < 0 ? -power : power);
            waitWhile(() -> abs(localizer.getPoseEstimate().getHeading() - initialPose.getHeading()) < abs(angleRadians), () -> {
                localizer.update();
            });
            stopAllMotors();
        }
    }

    /**
     * Causes the drivetrain to turn by a specific angle.
     *
     * @param power     The power to turn at.
     * @param angle     The angle to turn (Counterclockwise is positive, clockwise is negative).
     * @param angleUnit The units of the angle parameter.
     */
    public final void turnSimple(double power, double angle, @NotNull HALAngleUnit angleUnit) {
        turnSimple(power, angleUnit.convertTo(HALAngleUnit.RADIANS).apply(angle));
    }

    /**
     * Causes the drivetrain to turn to a specific angle using the turn-to-angle PID controller.
     *
     * @param targetAngle    The target angle to turn to (counterclockwise is positive, clockwise is negative).
     * @param angleUnit      The unit of the target angle.
     * @param angleTolerance The tolerance value of the target angle.
     * @param toleranceUnit  The units of the angleTolerance parameter.
     */
    public final void turnPID(double targetAngle, @NotNull HALAngleUnit angleUnit, double angleTolerance, @NotNull HALAngleUnit toleranceUnit) {
        turnController.setTargetPosition(angleUnit.convertTo(HALAngleUnit.RADIANS).apply(targetAngle));
        turnController.update(localizer.getPoseEstimate().getHeading());

        angleTolerance = toleranceUnit.convertTo(HALAngleUnit.RADIANS).apply(abs(angleTolerance));

        while (robot.opModeIsActive() && abs(turnController.getLastError()) > angleTolerance) {
            localizer.update();
            double correction = turnController.update(localizer.getPoseEstimate().getHeading());
            if (correction == 0) break;
            turnPowerInternal(correction);
        }
        stopAllMotors();
    }

    /**
     * Causes the drivetrain to turn to a specific angle using the turn-to-angle PID controller.
     *
     * @param targetAngle           The target angle to turn to (counterclockwise is positive, clockwise is negative).
     * @param angleUnit             The unit of the target angle.
     * @param angleToleranceRadians The tolerance value of the target angle in radians.
     */
    public final void turnPID(double targetAngle, HALAngleUnit angleUnit, double angleToleranceRadians) {
        turnPID(targetAngle, angleUnit, angleToleranceRadians, HALAngleUnit.RADIANS);
    }

    /**
     * Causes the drivetrain to turn to a specific angle using the turn-to-angle PID controller.
     *
     * @param targetAngleRadians    The target angle to turn to in radians (counterclockwise is positive, clockwise is negative).
     * @param angleToleranceRadians The tolerance value of the target angle in radians.
     */
    public final void turnPID(double targetAngleRadians, double angleToleranceRadians) {
        turnPID(targetAngleRadians, HALAngleUnit.RADIANS, angleToleranceRadians, HALAngleUnit.RADIANS);
    }

    /**
     * Causes the drivetrain to turn to a specific angle using the turn-to-angle PID controller.
     * Tolerance defaults to 1e-2 radians.
     *
     * @param targetAngle The target angle to turn to (counterclockwise is positive, clockwise is negative).
     * @param angleUnit   The unit of the target angle.
     */
    public final void turnPID(double targetAngle, HALAngleUnit angleUnit) {
        turnPID(targetAngle, angleUnit, 1e-2, HALAngleUnit.RADIANS);
    }

    /**
     * Causes the drivetrain to turn to a specific angle using the turn-to-angle PID controller.
     * Tolerance defaults to 1e-2 radians.
     *
     * @param targetAngleRadians The target angle to turn to in radians (counterclockwise is positive, clockwise is negative).
     */
    public final void turnPID(double targetAngleRadians) {
        turnPID(targetAngleRadians, HALAngleUnit.RADIANS);
    }

    /**
     * Sets the coefficients of the turn-to-angle pid controller.
     *
     * @param pidCoefficients The coefficients of the turn-to-angle PID controller.
     */
    public final void setTurnPID(PIDCoefficients pidCoefficients) {
        turnCoefficients = pidCoefficients;
        turnController = new PIDFController(turnCoefficients);
    }

    /**
     * Sets the value of the heading pid controller.
     *
     * @param pidCoefficients The coefficients of the heading PID controller.
     */
    public final void setHeadingPID(PIDCoefficients pidCoefficients) {
        headingCoefficients = pidCoefficients;
        headingController = new PIDFController(headingCoefficients);
    }

    /**
     * Sets the angle tolerance of the heading PID controller.
     *
     * @param tolerance     The tolerance of the heading PID controller.
     * @param toleranceUnit The unit for the tolerance parameter.
     */
    public final void setHeadingPIDTolerance(double tolerance, @NotNull HALAngleUnit toleranceUnit) {
        headingAngleToleranceRadians = toleranceUnit.convertTo(HALAngleUnit.RADIANS).apply(tolerance);
    }

    /**
     * Sets the angle tolerance of the heading PID controller.
     *
     * @param toleranceRadians The tolerance of the heading PID controller in radians.
     */
    public final void setHeadingPIDTolerance(double toleranceRadians) {
        setHeadingPIDTolerance(toleranceRadians, HALAngleUnit.RADIANS);
    }

    /**
     * Sets the scaling method for the drivetrain's velocity.
     *
     * @param velocityScaleMethod The scaling method for the drivetrain's velocity.
     */
    public final void setVelocityScaleMethod(SpeedScaleMethod velocityScaleMethod) {
        this.velocityScaleMethod = velocityScaleMethod;
    }

    /**
     * Sets the scaling method for the drivetrain's turn speed.
     *
     * @param turnSpeedScaleMethod The scaling method for the drivetrain's turn speed.
     */
    public final void setTurnSpeedScaleMethod(SpeedScaleMethod turnSpeedScaleMethod) {
        this.turnSpeedScaleMethod = turnSpeedScaleMethod;
    }

    /**
     * Sets a cap on the drivetrain's velocity.
     *
     * @param velocityCap The drivetrain's maximum desired velocity.
     */
    public final void setVelocityCap(double velocityCap) {
        if (velocityCap > 0) {
            this.velocityCap = Range.clip(velocityCap, 0, 1);
        }
    }

    /**
     * Sets a cap on the drivetrain's turn speed.
     *
     * @param turnSpeedCap The drivetrain's maximum desired turn speed.
     */
    public final void setTurnSpeedCap(double turnSpeedCap) {
        if (turnSpeedCap > 0) {
            this.turnSpeedCap = Range.clip(turnSpeedCap, 0, 1);
        }
    }

    /**
     * Sets a multiplier that will be applied to the drivetrain's velocity.
     *
     * @param velocityMultiplier A multiplier that will be applied to the drivetrain's velocity.
     */
    public final void setVelocityMultiplier(double velocityMultiplier) {
        constantSpeedMultiplier = abs(velocityMultiplier);
    }

    /**
     * Sets a multiplier that will be applied to the drivetrain's turn speed.
     *
     * @param turnSpeedMultiplier A multiplier that will be applied to the drivetrain's turn speed.
     */
    public final void setTurnSpeedMultiplier(double turnSpeedMultiplier) {
        constantTurnSpeedMultiplier = abs(turnSpeedMultiplier);
    }

    /**
     * Sets the angular velocity weight.
     *
     * @param angularVelocityWeight The angular velocity weight.
     */
    public final void setAngularVelocityWeight(double angularVelocityWeight) {
        OMEGA_WEIGHT = angularVelocityWeight;
    }

    /**
     * Sets the drivetrain's localizer. Note that this can be overriden if needed.
     *
     * @param localizer               The localizer for this drivetrain.
     * @param localizerCoordinateMode The type of coordinates the localizer will output (HAL or ROADRUNNER).
     */
    public void setLocalizer(Localizer localizer, CoordinateMode localizerCoordinateMode) {
        this.localizer = localizer;
        this.localizerCoordinateMode = localizerCoordinateMode;
    }

    /**
     * Gets the localizer used to track the robot's position.
     *
     * @return The localizer used to track the robot's position.
     */
    public final Localizer getLocalizer() {
        return localizer;
    }

    /**
     * Sets the drivetrain's localizer. Note that this can be overriden if needed.
     *
     * @param localizer The localizer for this drivetrain (defaults to assuming HAL coordinate outputs).
     */
    public final void setLocalizer(Localizer localizer) {
        setLocalizer(localizer, CoordinateMode.HAL);
    }

    /**
     * Sets the drivetrain coordinate mode (which changes whether entered commands are interpreted using the roadrunner or HAL coordinate systems).
     *
     * @param coordinateMode The desired drivetrain coordinate mode.
     */
    public final void setCoordinateMode(CoordinateMode coordinateMode) {
        this.coordinateMode = coordinateMode;
    }

    /**
     * Uses the localizer to get an estimate of the drivetrain's position.
     *
     * @return An estimate of the drivetrain's position.
     */
    @NotNull
    public final Pose2d getPoseEstimate() {
        return localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseEstimate());
    }

    /**
     * Sets the localizer pose estimate.
     *
     * @param poseEstimate The localizer pose estimate.
     */
    public final void setPoseEstimate(Pose2d poseEstimate) {
        localizer.setPoseEstimate(coordinateMode.convertTo(localizerCoordinateMode).apply(poseEstimate));
    }

    /**
     * Gets the localizer estimated pose velocity.
     *
     * @return The localizer estimated pose velocity.
     */
    public final Pose2d getPoseVelocity() {
        return localizerCoordinateMode.convertTo(coordinateMode).apply(localizer.getPoseVelocity());
    }

    /**
     * Updates the localizer.
     */
    public final void updateLocalizer() {
        localizer.update();
    }

    /**
     * The method used to scale drivetrain velocities or turn speeds.
     */
    public enum SpeedScaleMethod {
        //Do not scale
        NONE((Vector2D velocity) -> {
        }),
        //Square the velocity magnitude (but keep the direction).
        SQUARE((Vector2D velocity) -> {
            if (!velocity.isZeroVector()) {
                double magnitude = velocity.magnitude();
                velocity.normalize().multiply(magnitude * magnitude);
            }
        }),
        //Cube the velocity magnitude.
        CUBIC((Vector2D velocity) -> {
            if (!velocity.isZeroVector()) {
                double magnitude = velocity.magnitude();
                velocity.normalize().multiply(magnitude * magnitude * magnitude);
            }
        });

        //The function used to scale the velocity.
        public final Consumer<Vector2D> scaleFunction;

        /**
         * The constructor for the SpeedScaleMethod enum.
         *
         * @param scaleFunction The function used to scale the velocity.
         */
        SpeedScaleMethod(Consumer<Vector2D> scaleFunction) {
            this.scaleFunction = scaleFunction;
        }
    }
}