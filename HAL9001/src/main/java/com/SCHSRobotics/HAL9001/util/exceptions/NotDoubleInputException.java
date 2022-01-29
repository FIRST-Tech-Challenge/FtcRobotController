package com.SCHSRobotics.HAL9001.util.exceptions;

import org.jetbrains.annotations.Nullable;

/**
 * An exception thrown when a button meant to return double data is mapped to a button that does not return double data.
 * <p>
 * Creation Date: 7/20/19
 *
 * @author Dylan Zueck, Crow Force
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.util.control.Button
 * @see com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad
 * @see RuntimeException
 * @since 1.0.0
 */
public class NotDoubleInputException extends RuntimeException {

    /**
     * Constructor for NotDoubleInputException.
     *
     * @param message The message to print to the screen.
     */
    public NotDoubleInputException(@Nullable String message) {
        super(message);
    }
}