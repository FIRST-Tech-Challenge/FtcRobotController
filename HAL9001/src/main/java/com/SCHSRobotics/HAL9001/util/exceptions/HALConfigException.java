package com.SCHSRobotics.HAL9001.util.exceptions;

/**
 * An exception thrown when there is a problem with HAL's config system.
 * <p>
 * Creation Date: 9/29/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.system.config.HALConfig
 * @see RuntimeException
 * @since 1.1.0
 */
public class HALConfigException extends RuntimeException {

    /**
     * Constructor for HALConfigException.
     *
     * @param message The error message to print to the screen.
     */
    public HALConfigException(String message) {
        super(message);
    }
}
