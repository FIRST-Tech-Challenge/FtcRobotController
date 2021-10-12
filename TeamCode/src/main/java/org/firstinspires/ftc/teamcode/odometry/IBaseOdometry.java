package org.firstinspires.ftc.teamcode.odometry;

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
    int getCurrentX();

    /**
     * @return the current Y coordinate in inches
     */
    int getCurrentY();

    /**
     * @return the current robot heading in degrees
     */
    int getCurrentHeading();
}
