package com.SCHSRobotics.HAL9001.system.config;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation used to give HAL subsystems specific names. Used to annotate subsystem fields in the robot class
 * <p>
 * Creation Date: 4/11/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see DisableSubSystem
 * @see com.SCHSRobotics.HAL9001.system.robot.Robot
 * @see com.SCHSRobotics.HAL9001.system.robot.SubSystem
 * @since 1.1.0
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface ConfigLabel {

    /**
     * The id of the attached subsystem.
     *
     * @return The id of the attached subsystem.
     */
    String label();
}
