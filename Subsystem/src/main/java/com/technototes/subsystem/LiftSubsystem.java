package com.technototes.subsystem;

/** Interface for lift subsystems
 * @author Alex Stedman
 */
public interface LiftSubsystem extends PositionalSubsystem {
    /** Sends lift up one increment
     *
     */
    default void up(){
        goToPosition(Math.min(getPosition(), getMaxPosition()));
    }

    /** Sends lift down one increment
     *
     */
    default void down(){
        goToPosition(Math.max(getPosition(), getMinPosition()));
    }

    /** Sends lift to the top of its range
     *
     */
    default void goToTop(){
        goToPosition(getMaxPosition());
    }

    /** Sends lift to the bottom of its range
     *
     */
    default void goToBottom(){
        goToPosition(getMinPosition());
    }
}
