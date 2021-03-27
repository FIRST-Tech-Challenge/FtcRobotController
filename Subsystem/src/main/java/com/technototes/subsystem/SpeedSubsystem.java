package com.technototes.subsystem;

/** An interface for subsystems with variable speed
 * @author Alex Stedman
 */
public interface SpeedSubsystem {
    /** Set the speed of the subsystem
     *
     * @param speed The speed
     */
    void setSpeed(double speed);

    /** Get the current subsystem speed
     *
     * @return The current speed
     */
    double getSpeed();

    /** Stops the subsystem
     *
     */
    default void stop(){
        setSpeed(0);
    }
}
