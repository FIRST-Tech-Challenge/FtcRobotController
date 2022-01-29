package com.SCHSRobotics.HAL9001.util.exceptions;

import org.jetbrains.annotations.Nullable;

/**
 * An exception that is thrown when things cannot be converted into other things or made out of nothing.
 * <p>
 * Creation Date: 7/19/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see RuntimeException
 * @since 1.0.0
 */
public class NotAnAlchemistException extends RuntimeException {

    /**
     * Constructor for NotAnAlchemistException.
     *
     * @param message The error message to print to the screen.
     */
    public NotAnAlchemistException(@Nullable String message) {
        super(message);
    }
}