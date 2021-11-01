package org.firstinspires.ftc.teamcode.odometry;

import android.graphics.Point;

import org.firstinspires.ftc.teamcode.bots.BotMoveRequest;

/**
 * A runnable interface that has basic odometry methods
 */
public interface IBaseOdometry extends Runnable {

    /**
     * Set the initial position and reset internal counters
     * @param startXInches start X coordinates in inches
     * @param startYInches start Y coordinates in inches
     * @param startHeadingDegrees start robot heading in degrees
     * @throws Exception if anything goes wrong
     */
    void setInitPosition(int startXInches, int startYInches, int startHeadingDegrees) throws Exception;

    /**
     * Stop the thread and stop collecting odometery
     */
    void stop();

    /**
     * @return the current X coordinate in inches
     */
    double getCurrentX();

    /**
     * @return the current Y coordinate in inches
     */
    double getCurrentY();

     void reverseHorEncoder();

    void setPersistPosition(boolean persistPosition);

    void init(Point startPos, double initialOrientation);

    double getInitialOrientation();

    /**
     * @return the current robot heading in degrees
     */
    double getOrientation();

    /**
     * @return the current robot positive heading in degrees (0 to 360)
     */
    double getAdjustedCurrentHeading();

    int getThreadSleepTime();

    void setTarget(BotMoveRequest target);

    double getRealSpeedLeft();

    double getRealSpeedRight();

    boolean isLeftLong();

    boolean isTrackingInitialized();

}
