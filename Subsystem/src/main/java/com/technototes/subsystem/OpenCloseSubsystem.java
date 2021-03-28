package com.technototes.subsystem;

/** Interface for 2 positional subsystems that open and close
 * @author Alex Stedman
 */
public interface OpenCloseSubsystem {
    /** The state for the subsystem
     *
     */
    enum State{
        OPENED, CLOSED
    }

    /** Return the state
     *
     * @return The state
     */
    State getState();

    /** Open
     *
     */
    void open();

    /** Close
     *
     */
    void close();
}
