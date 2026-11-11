package org.firstinspires.ftc.teamcode.Resources;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TrajectoryKinematics

 * Utility class used to estimate launcher ramp angle and flywheel speed needed to score
 * into the goal from a measured distance. This class uses simple regression functions
 * (tuned from empirical testing) to convert a distance in inches to:
 * - an initial ramp angle in degrees
 * - a flywheel speed (magnitude) in degrees per second

 * Important notes and assumptions:
 * - All distances are in inches unless otherwise noted.
 * - Angles are in degrees.
 * - Flywheel "magnitude" is treated as degrees per second (an angular velocity command
 *   for the flywheel motor controller).
 * - The regression functions here are based on limited prototype testing. They are
 *   intended as simple mapping functions and should be re-tuned with larger datasets
 *   for production use.
 * - This class does not perform physical ballistic calculations. Instead it returns
 *   values from tuned regression polynomials (see {@link #angleRegression} and
 *   {@link #magnitudeRegression}).
 *
 * Thread-safety: This class is not synchronized. If accessed concurrently from multiple
 * threads (for example multiple opmodes), callers should ensure external synchronization
 * or use separate instances.
 */
@Config
public class TrajectoryKinematics {
    // A list of example sample distances/angles/magnitudes used for deriving the
    // regression functions (kept here for reference). These arrays are not used
    // directly by the calculation methods below but represent the tuning dataset.
    // NOTE: These were collected from early-season prototype testing and should be
    // replaced with larger, regularly spaced data (for example every 5 inches).
    @SuppressWarnings("unused")
    private final double[] distances = {
            24,
            81,
            94,
            120,
    };

    @SuppressWarnings("unused")
    private final double[] angles = {
            20,
            35,
            35,
            35
    };
    @SuppressWarnings("unused")
    private final double[] magnitudes = {
            1100,
            1400,
            1550,
            1750,
    };

    // The computed initial ramp angle (degrees).
    private double initialAngle = 0;
    // The computed flywheel speed (degrees per second).
    private double launchMagnitude = 0;

    // Corrections applied to the goal coordinates to compensate for robot motion.
    // These offsets are in the same units as goal coordinates (inches).
    private double targetCorrectionXMag, targetCorrectionYMag, targetCorrectionXAngle,targetCorrectionYAngle;

    // Scalars that convert robot velocity (units depend on caller) into a positional
    // correction for the target. Typical usage: updateVelocities() multiplies the
    // provided velocity by these scalars to compute a small offset to the aim point.
    // The units and meaning of velocityX/velocityY should be consistent with the
    // chosen scalars (for example inches per second * seconds-of-lag -> inches).
    public static double velocityScalarXAngle = 0.62;
    public static double velocityScalarYAngle = 0.62;
    public static double velocityScalarXMag = 0.925;
    public static double velocityScalarYMag = 0.7;

    public static double goalX = -59.2;
    public static double goalY = 63.0;
    public static double flywheelErrorToAngle = -0.03;
    private double flywheelError;

    private Telemetry telemetry;

    /**
     * Create a TrajectoryKinematics instance.
     *
     * @param isAuto true if the instance will be used in autonomous mode. Currently
     *               this parameter is accepted for API compatibility but not used.
     */
    public TrajectoryKinematics(boolean isAuto, Telemetry telemetry){
        // Intentionally left blank. Previously there was an autonomous adjustment
        // variable here; it was removed because it was not used. Keep the
        // constructor argument for compatibility with existing callers.
        this.telemetry = telemetry;
    }

    // the getBearingToTag is used to turn the robot so it is facing the center of the tag
    /**
     * Compute a heading (bearing) that points the robot toward the center of the goal
     * (april tag). This function converts the robot's (x,y) position and the selected
     * alliance color into an absolute bearing (degrees) that the robot should face.
     *
     * Notes on units and coordinate system:
     * - posX / posY are the robot's position in the same coordinate system as the
     *   goalX/goalY constants (inches).
     * - The returned heading is in degrees and includes a fixed coordinate correction
     *   offset for the robot's mounting and components.
     *
     * @param mode "red" or "blue" alliance; selects a different goal coordinate.
     * @param isAuto not currently used inside the method (kept for API compatibility).
     * @param x robot X position (inches)
     * @param y robot Y position (inches)
     * @return heading in degrees the robot should face to aim at the goal
     */
    @SuppressWarnings("unused")
    public double getBearingToTag(String mode, Boolean isAuto, double x, double y) {
        double angle;
        // coordinate correction applied to the final output to align robot coordinate frame
        double coordinate_correction_offset = 90;
        // small angular offset to account for the flywheel being offset from the robot center
        double FLYWHEEL_OFFSET = 0;
        double tempGoalX = goalX;
        double tempGoalY = goalY;


        switch (mode) {
            case "red":
                // These are empirically set goal coordinates (inches) for the red alliance
                tempGoalY = 52;
                tempGoalX = -64.2;
                if (isAuto) {
                    tempGoalY = 50;
                    tempGoalX = -60.2;
                }
                // Geometry: Math.atan(5/123.5) represents a small angular offset due to
                // the flywheel's vertical/horizontal displacement relative to the robot
                // center. The numbers are empirical and should be documented in design notes.
                FLYWHEEL_OFFSET = Math.toDegrees(Math.atan(5 / 123.5));
                break;
            case "blue":
                // Empirically determined goal coordinates (inches) for the blue alliance
                tempGoalY *= -1;
                FLYWHEEL_OFFSET = Math.toDegrees(Math.atan(5 / 123.5));
                break;
        }

        // Apply small corrections that compensate for robot motion or measurement bias.
        tempGoalX += targetCorrectionXAngle;
        tempGoalY += targetCorrectionYAngle;

        double x_difference = x - tempGoalX;
        double y_difference = y - tempGoalY;


        // atan2 arguments are (y, x) but here we want the bearing relative to robot axes.
        // Convert to degrees and offset to match the robot's heading convention.
        angle = 90 + Math.toDegrees(Math.atan2(y_difference, x_difference));
        return angle - FLYWHEEL_OFFSET + coordinate_correction_offset;
    }

    public double getDistance(String mode, double x, double y){
        double distance = 0.0;
        double posX = x;
        double posY = y;
        double tempGoalX = goalX;
        double tempGoalY = goalY;
        switch (mode) {
            case "red":
                // These are empirically set goal coordinates (inches) for the red alliance

                break;
            case "blue":
                // Empirically determined goal coordinates (inches) for the blue alliance

                tempGoalY *= -1;
                break;
        }
        tempGoalX += targetCorrectionXMag;
        tempGoalY += targetCorrectionYMag;
        distance = Math.hypot(tempGoalX-posX,tempGoalY-posY);
        return distance;
    }

    public static double overrideSpeed = 0;
    public static double overrideRamp = 0;

    /**
     * Updates the computed trajectory values (initialAngle and launchMagnitude) using
     * the tuned regression functions. Call this before retrieving values via
     * {@link #getInitialAngle} and {@link #getLaunchMagnitude}.
     *
     * @param distanceInches distance from front of robot to april tag (inches)
     */
    public void calculateTrajectory(double distanceInches, double flywheelError) {

        // All regression functions are calculated using the stored values above and put into Desmos graphing calculator to create a fourth degree regression function!
        initialAngle = angleRegression(distanceInches) - necessaryRampOffset(flywheelError);
        launchMagnitude = magnitudeRegression(distanceInches);

        telemetry.addLine("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
        telemetry.addData("Distance", distanceInches);
        telemetry.addData("Ramp angle", initialAngle);
        telemetry.addData("Magnitude", launchMagnitude);
        telemetry.addLine("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    }

    /**
     * Regression mapping from distance (inches) to launcher ramp angle (degrees).
     * Implementation detail: this is a simple linear fit (tuned empirically). If
     * you expect distances outside the measured range, re-tune the regression and/or
     * clamp values appropriately.
     *
     * @param distance Inches from front (camera side) of the robot to the april tag
     * @return ramp angle in degrees
     */
    private double angleRegression(double distance){


        // values a-e represent the tuning values of this 1st-degree polynomial
        double a = 1.17842026e-09;
        double b = -6.19528564e-07;
        double c = 1.27750841e-04;
        double d = -1.30363270e-02;
        double e = 6.85788568e-01;
        double f = -1.79995933e+01;
        double g = 2.59766158e+02;
        double result = a * Math.pow(distance,6) + b * Math.pow(distance,5) + c * Math.pow(distance,4) + d * Math.pow(distance,3) + e * Math.pow(distance,2) + f * Math.pow(distance,1) + g;

        return Math.min(result, 84);
    }

    /**
     * Update the target corrections from measured robot velocity. The method uses
     * small scalar factors (see {@link #velocityScalarXMag} and {@link #velocityScalarYMag})
     * to convert velocity into a positional offset for the aim point.
     *
     * Example: if the robot is moving forward, the aim should lead the target in the
     * direction of motion to compensate for travel during the projectile's flight time.
     *
     * @param velocityX robot velocity along X (units depend on caller; scalars convert to inches)
     * @param velocityY robot velocity along Y (units depend on caller; scalars convert to inches)
     */
    public void updateVelocities(double velocityX, double velocityY){
        targetCorrectionXMag = -velocityX * velocityScalarXMag;
        targetCorrectionYMag = -velocityY * velocityScalarYMag;
        targetCorrectionXAngle = -velocityX * velocityScalarXAngle;
        targetCorrectionYAngle = -velocityY * velocityScalarYAngle;
    }

    /**
     * Regression mapping from distance (inches) to flywheel angular speed
     * (degrees per second). This polynomial was tuned from prototype testing.
     *
     * Implementation details:
     * - The polynomial uses a cubic term plus quadratic, linear, and constant terms.
     * - If you change units for distance, update these coefficients accordingly.
     *
     * @param distance Inches from front (camera side) of the robot to the april tag.
     * @return flywheel speed in degrees per second
     */
    private double magnitudeRegression(double distance){
        //quadratic tuning values

        double a = 3.66673556e-07;
        double b = -1.56556627e-04;
        double c = 2.47391956e-02;
        double d = -1.76371903;
        double e = 5.94863351e+01;
        double f = 2.84088580e+02;

        double result = a * Math.pow(distance,5) + b * Math.pow(distance,4) + c * Math.pow(distance,3) + d * Math.pow(distance,2) + e * Math.pow(distance,1) + f;

        // Math.pow is the exponent function, this is a second degree polynomial that is tuning based on real world testing
        return Math.min(result, 1600);
    }

    /**
     * Getter for initial angle (degrees). Call {@link #calculateTrajectory} first.
     *
     * @return computed ramp angle in degrees
     */
    public double getInitialAngle(){
          return initialAngle;
    }

    /**
     * getter for the flywheel speed (degrees per second). Call {@link #calculateTrajectory} first.
     *
     * @return computed flywheel angular velocity (degrees/sec)
     */
    public double getLaunchMagnitude(){
               return launchMagnitude;
    }

    /**
     * This cleans up the calculate trajectories to help it calculate the necessary ramp offset due to flywheel error
     * @return
     */
    public double necessaryRampOffset(double flywheelError){

        return Math.abs(flywheelError)*flywheelErrorToAngle;
    }
}