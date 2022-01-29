package com.SCHSRobotics.HAL9001.system.config;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * A class annotation used to easily put into place program configuration options.
 * <p>
 * Creation Date: 12/18/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.system.robot.LinkTo
 * @see HALConfig
 * @see com.SCHSRobotics.HAL9001.system.robot.HALProgram
 * @since 1.0.6
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface ProgramOptions {
    /**
     * A list of enum classes that are being used as configurable options for the attached opmode.
     *
     * @return A list of enum classes that are being used as configurable options for the attached opmode.
     */
    Class<? extends Enum<?>>[] options();
}
