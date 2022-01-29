package com.SCHSRobotics.HAL9001.system.robot;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation that allows for the easy creation of robots in HAL Programs.
 * <p>
 * Creation Date: 12/23/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see HALProgram
 * @see Robot
 * @since 1.0.6
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface MainRobot {}