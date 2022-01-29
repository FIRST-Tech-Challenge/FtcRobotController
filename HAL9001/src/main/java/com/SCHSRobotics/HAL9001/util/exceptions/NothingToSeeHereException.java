package com.SCHSRobotics.HAL9001.util.exceptions;

import org.jetbrains.annotations.Nullable;

/**
 * An exception thrown if something needed is missing.
 * <p>
 * Creation Date: 10/9/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see RuntimeException
 * @since 1.0.0
 */
public class NothingToSeeHereException extends RuntimeException {

    /**
     * Constructor for NothingToSeeHereException.
     *
     * @param message The message to print to the screen.
     */
    public NothingToSeeHereException(@Nullable String message) {
        super(message);
    }
}