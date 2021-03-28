package com.technototes.subsystem;

/** An interface for flywheel subsystems
 * @author Alex Stedman
 */
public interface FlywheelSubsystem extends SpeedSubsystem {
    /** Set idle speed for flywheel
     *
     * @param speed The idle speed
     */
    void setIdleSpeed(double speed);

}
