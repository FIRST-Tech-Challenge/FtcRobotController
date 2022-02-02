package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry.enums.FieldPoints;

/**
 * A interface that standardizes methods to get the current robot location.
 * Location must be 0,0,0 on initialization
 * Location must be returned in inches
 * Location methods must be thread safe to call
 */
public interface LocalizationAlgorithm {

    /**
     * Returns the X (inches) ,Y (inches) and Rotation (degrees) values of the robot
     *
     * @return X, Y and Rotation
     */
    double[] getPos();

    /**
     * Gets the x position of the robot in inches
     * (0,0) is relative to the red warehouse side of the field
     *
     * @return The x position of the robot in inches
     */
    double getX();

    /**
     * Gets the y position of the robot in inches
     * (0,0) is relative to the red warehouse side of the field
     *
     * @return The y position of the robot in inches
     */
    double getY();

    /**
     * Gets the orientation of the robot in degrees
     * north is pointing towards center and is perpendicular to the warehouse wall
     *
     * @return The heading of the robot in degrees
     */
    double getRot();

    /**
     * Sets the position of the robot
     *
     * @param X   The x coordinate the robot is at
     * @param Y   The Y coordinate the robot is at
     * @param rot The heading the robot is at
     * @throws InterruptedException Throws if the thread is interrupted while setting position
     */
    void setPos(double X, double Y, double rot) throws InterruptedException;

    /**
     * Sets the position of the robot using an Enum key from FieldPoints
     *
     * @param initPos the enum key of a three value array of an init position
     * @throws InterruptedException Throws if the Thread is interupted while waiting for lock. If is interrupted, values are not changed
     */
    void setPos(FieldPoints initPos) throws InterruptedException;
}
