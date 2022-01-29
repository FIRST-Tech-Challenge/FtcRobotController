package com.SCHSRobotics.HAL9001.util.exceptions;

/**
 * An exception thrown when there is a problem with HAL's GUI system.
 * <p>
 * Creation Date: 9/29/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.system.gui.HALGUI
 * @see RuntimeException
 * @since 1.1.0
 */
public class HALGUIException extends RuntimeException {

    /**
     * Constructor for HALGUIException.
     *
     * @param message The error message to print to the screen.
     */
    public HALGUIException(String message) {
        super(message);
    }
}
