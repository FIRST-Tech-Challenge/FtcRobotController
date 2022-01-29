package com.SCHSRobotics.HAL9001.util.exceptions;

import org.jetbrains.annotations.Nullable;

/**
 * An exception thrown if an invalid link is used in the LinkTo annotation.
 * <p>
 * Creation Date: 12/19/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.system.robot.LinkTo
 * @see RuntimeException
 * @since 1.0.0
 */
public class InvalidLinkException extends RuntimeException {

    /**
     * Constructor for InvalidLinkException.
     *
     * @param message The error message to print to the screen.
     */
    public InvalidLinkException(@Nullable String message) {
        super(message);
    }
}