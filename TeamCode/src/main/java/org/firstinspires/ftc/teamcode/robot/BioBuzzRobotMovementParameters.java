package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.broombots.sweep.Classes.RobotMovementParameters;
import com.broombots.sweep.Defaults.MecanumDriveAcceleration;

import org.ejml.simple.SimpleMatrix;

@Config
/** Provides baseline motion limits and tunable PID gains for a mecanum-drive robot. */
public class BioBuzzRobotMovementParameters implements RobotMovementParameters {
    /** Assumed robot mass in kilograms; retained for API compatibility, not used by the fixed limits. */
    private double robotMass;
    /** Maximum linear acceleration in inches per second squared. */
    public static double robotTopAcceleration = 25;
    /** Maximum angular acceleration in degrees per second squared. */
    public static double robotTopAngularAcceleration = 10;
    /** Maximum angular velocity in degrees per second. */
    public static double robotTopAngularVelocity = 30;
    /** Base speed value used by the mecanum maximum-speed calculation, in inches per second. */
    public static double robotTopSpeedStraight = 40;

    /** Public static for FTC Dashboard tuning; x/y position proportional gain (units follow the controller input and output). */
    public static double standardPositionP = 0.95;
    /** Public static for FTC Dashboard tuning; x/y position integral gain (units follow the controller input and output). */
    public static double standardPositionI = 0.12;
    /** Public static for FTC Dashboard tuning; x/y position derivative gain (units follow the controller input and output). */
    public static double standardPositionD = 0.01;
    /** Public static for FTC Dashboard tuning; heading proportional gain (units follow the controller input and output). */
    public static double rotationP = 0.07;
    /** Public static for FTC Dashboard tuning; heading integral gain (units follow the controller input and output). */
    public static double rotationI = 0.02;
    /** Public static for FTC Dashboard tuning; heading derivative gain (units follow the controller input and output). */
    public static double rotationD = 0.0005;
    /** Public static for FTC Dashboard tuning; x/y velocity proportional gain (units follow the controller input and output). */
    public static double velocityP = 0.09;
    /** Public static for FTC Dashboard tuning; x/y velocity integral gain (units follow the controller input and output). */
    public static double velocityI = 0.004;
    /** Public static for FTC Dashboard tuning; x/y velocity derivative gain (units follow the controller input and output). */
    public static double velocityD = 0.001;

    /** Creates movement parameters using a specified robot mass.
     * @param robotMass robot mass in kilograms
     */
    public BioBuzzRobotMovementParameters(double robotMass){
        this.robotMass = robotMass;
    }
    /** Creates movement parameters with a 10 kg assumed robot mass. */
    public BioBuzzRobotMovementParameters(){
        robotMass = 10;
    }

    /** {@inheritDoc} */
    public double getMaxVelocity(double direction){
        return robotTopSpeedStraight * MecanumDriveAcceleration.getMovementMagnitude(direction) * (Math.sqrt(2) / 2);
    }
    /** {@inheritDoc} */
    public double getMaxAngleVelocity(){
        return robotTopAngularVelocity;
    }
    /** {@inheritDoc} */
    public double getMaxStableAcceleration(double direction){
        return robotTopAcceleration * MecanumDriveAcceleration.getMovementMagnitude(direction);
    }
    /** {@inheritDoc} */
    public double getMaxStableAngularAcceleration(double movementMagnitude, boolean clockwise){
        return robotTopAngularAcceleration;
    }
    /** {@inheritDoc} */
    @Override
    public double getAngleFullPowerToErrorThreshold() {
        return 40;
    }

    /** {@inheritDoc} */
    @Override
    public SimpleMatrix getPIDCoefficients(){
        SimpleMatrix coeffs = new SimpleMatrix(3,3);
        coeffs.set(0,0, standardPositionP);
        coeffs.set(0,1,standardPositionI);
        coeffs.set(0,2,standardPositionD);

        coeffs.set(1,0, rotationP);
        coeffs.set(1,1,rotationI);
        coeffs.set(1,2,rotationD);

        coeffs.set(2,0,velocityP);
        coeffs.set(2,1,velocityI);
        coeffs.set(2,2,velocityD);

        return coeffs;
    }
}
