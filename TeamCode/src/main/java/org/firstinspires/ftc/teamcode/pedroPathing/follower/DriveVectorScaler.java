package org.firstinspires.ftc.teamcode.pedroPathing.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 * This is the DriveVectorScaler class. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction and returns an array with wheel powers.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
public class DriveVectorScaler {
    // This is ordered left front, left back, right front, right back. These are also normalized.
    private Vector[] mecanumVectors;
    private double maxPowerScaling = 1;

    /**
     * This creates a new DriveVectorScaler, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     *
     * @param frontLeftVector this is the front left mecanum wheel's preferred drive vector.
     */
    public DriveVectorScaler(Vector frontLeftVector) {
        Vector copiedFrontLeftVector = MathFunctions.normalizeVector(frontLeftVector);
        mecanumVectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};
    }

    /**
     * This takes in vectors for corrective power, heading power, and pathing power and outputs
     * an Array of four doubles, one for each wheel's motor power.
     *
     * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
     *
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bezier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heaing, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     * @return this returns an Array of doubles with a length of 4, which contains the wheel powers.
     */
    public double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);

        // the powers for the wheel vectors
        double [] wheelPowers = new double[4];

        // This contains a copy of the mecanum wheel vectors
        Vector[] mecanumVectorsCopy = new Vector[4];

        // this contains the pathing vectors, one for each side (heading control requires 2)
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            // checks for corrective power equal to max power scaling in magnitude. if equal, then set pathing power to that
            truePathingVectors[0] = MathFunctions.copyVector(correctivePower);
            truePathingVectors[1] = MathFunctions.copyVector(correctivePower);
        } else {
            // corrective power did not take up all the power, so add on heading power
            Vector leftSideVector = MathFunctions.subtractVectors(correctivePower, headingPower);
            Vector rightSideVector = MathFunctions.addVectors(correctivePower, headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                //if the combined corrective and heading power is greater than 1, then scale down heading power
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower), findNormalizingScaling(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, -1)));
                truePathingVectors[0] = MathFunctions.subtractVectors(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor));
                truePathingVectors[1] = MathFunctions.addVectors(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor));
            } else {
                // if we're here then we can add on some drive power but scaled down to 1
                Vector leftSideVectorWithPathing = MathFunctions.addVectors(leftSideVector, pathingPower);
                Vector rightSideVectorWithPathing = MathFunctions.addVectors(rightSideVector, pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    // too much power now, so we scale down the pathing vector
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower), findNormalizingScaling(rightSideVector, pathingPower));
                    truePathingVectors[0] = MathFunctions.addVectors(leftSideVector, MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
                    truePathingVectors[1] = MathFunctions.addVectors(rightSideVector, MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
                } else {
                    // just add the vectors together and you get the final vector
                    truePathingVectors[0] = MathFunctions.copyVector(leftSideVectorWithPathing);
                    truePathingVectors[1] = MathFunctions.copyVector(rightSideVectorWithPathing);
                }
            }
        }

        truePathingVectors[0] = MathFunctions.scalarMultiplyVector(truePathingVectors[0], 2.0);
        truePathingVectors[1] = MathFunctions.scalarMultiplyVector(truePathingVectors[1], 2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectorsCopy[i] = MathFunctions.copyVector(mecanumVectors[i]);

            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent()*truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent()*mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent()*truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent()*mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent()*truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent()*mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent()*truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent()*mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent());

        double wheelPowerMax = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])), Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));
        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] /= wheelPowerMax;
            wheelPowers[1] /= wheelPowerMax;
            wheelPowers[2] /= wheelPowerMax;
            wheelPowers[3] /= wheelPowerMax;
        }

        return wheelPowers;
    }

    /**
     * This takes in two Vectors, one static and one variable, and returns the scaling factor that,
     * when multiplied to the variable Vector, results in magnitude of the sum of the static Vector
     * and the scaled variable Vector being the max power scaling.
     *
     * IMPORTANT NOTE: I did not intend for this to be used for anything other than the method above
     * this one in this class, so there will be errors if you input Vectors of length greater than maxPowerScaling,
     * and it will scale up the variable Vector if the magnitude of the sum of the two input Vectors
     * isn't greater than maxPowerScaling. So, just don't use this elsewhere. There's gotta be a better way to do
     * whatever you're trying to do.
     *
     * I know that this is used outside of this class, however, I created this method so I get to
     * use it if I want to. Also, it's only used once outside of the DriveVectorScaler class, and
     * it's used to scale Vectors, as intended.
     *
     * @param staticVector the Vector that is held constant.
     * @param variableVector the Vector getting scaled to make the sum of the input Vectors have a
     *                       magnitude of maxPowerScaling.
     * @return returns the scaling factor for the variable Vector.
     */
    public double findNormalizingScaling(Vector staticVector, Vector variableVector) {
            double a = Math.pow(variableVector.getXComponent(), 2) + Math.pow(variableVector.getYComponent(), 2);
            double b = staticVector.getXComponent() * variableVector.getXComponent() + staticVector.getYComponent() * variableVector.getYComponent();
            double c = Math.pow(staticVector.getXComponent(), 2) + Math.pow(staticVector.getYComponent(), 2) - Math.pow(maxPowerScaling, 2);
            return (-b + Math.sqrt(Math.pow(b, 2) - a*c))/(a);

    }

    /**
     * Sets the maximum power that can be used by the drive vector scaler. Clamped between 0 and 1.
     *
     * @param maxPowerScaling setting the max power scaling
     */
    public void setMaxPowerScaling(double maxPowerScaling) {
        this.maxPowerScaling = MathFunctions.clamp(maxPowerScaling, 0, 1);
    }

    /**
     * Gets the maximum power that can be used by the drive vector scaler. Ranges between 0 and 1.
     *
     * @return returns the max power scaling
     */
    public double getMaxPowerScaling() {
        return maxPowerScaling;
    }
}
