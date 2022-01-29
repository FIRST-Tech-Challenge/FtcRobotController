package com.SCHSRobotics.HAL9001.system.robot;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation that links two HAL programs together, both in the config and via an auto-transitioner.
 * <p>
 * Creation Date: 12/19/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see com.SCHSRobotics.HAL9001.util.control.AutoTransitioner
 * @see Robot
 * @see HALProgram
 * @since 1.0.6
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface LinkTo {

    /**
     * The destination opmode's name (as specified in @TeleOp or @Autonomous).
     *
     * @return The destination opmode's name (as specified in @TeleOp or @Autonomous).
     * @see com.qualcomm.robotcore.eventloop.opmode.Autonomous
     * @see com.qualcomm.robotcore.eventloop.opmode.TeleOp
     */
    String destination();

    /**
     * Whether or not to automatically transition to the linked opmode at the end of the current opmode.
     *
     * @return Whether or not to automatically transition to the linked opmode at the end of the current opmode.
     * @see com.SCHSRobotics.HAL9001.util.control.AutoTransitioner
     */
    boolean auto_transition() default true;
}