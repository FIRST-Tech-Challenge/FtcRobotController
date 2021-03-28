package com.technototes.subsystem;

/** A subsystem for things with multiple positions and setpoints
 * @author Alex Stedman
 */
public interface PositionalSubsystem extends SpeedSubsystem {
    /** Get the maximum position for subsystem
     *
     * @return The maximum position
     */
    int getMaxPosition();

    /** Get the minimum position for subsystem
     *
     * @return The minimum position
     */
    default int getMinPosition(){
        return 0;
    }

    /** Get the current position
     *
     * @return The current position
     */
    int getPosition();

    /** Go to a position
     *
     * @param position The position to go to
     */
    void goToPosition(int position);
}
