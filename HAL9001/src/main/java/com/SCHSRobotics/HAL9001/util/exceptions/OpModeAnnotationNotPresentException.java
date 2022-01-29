package com.SCHSRobotics.HAL9001.util.exceptions;

import org.jetbrains.annotations.Nullable;

/**
 * An exception thrown if an @TeleOp or @Autonomous annotation is missing from an opmode being processed.
 * <p>
 * Creation Date: 9/29/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.system.robot.HALProgram
 * @see com.SCHSRobotics.HAL9001.system.robot.Robot
 * @see RuntimeException
 * @since 1.1.0
 */
public class OpModeAnnotationNotPresentException extends RuntimeException {

    /**
     * Constructor for NothingToSeeHereException.
     *
     * @param message The message to print to the screen.
     */
    public OpModeAnnotationNotPresentException(@Nullable String message) {
        super(message);
    }
}