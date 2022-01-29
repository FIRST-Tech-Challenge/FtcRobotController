package com.SCHSRobotics.HAL9001.util.exceptions;

import org.jetbrains.annotations.Nullable;

/**
 * An exception that is thrown when a button meant to return vector data is mapped to a button that does not return vector data.
 * <p>
 * Creation Date: 8/31/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.util.control.Button
 * @see com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad
 * @see RuntimeException
 * @since 1.0.0
 */
public class NotVectorInputException extends RuntimeException {

    /**
     * Constructor for NotVectorInputException.
     *
     * @param message The error message to print to the screen.
     */
    public NotVectorInputException(@Nullable String message) {
        super(message);
    }
}