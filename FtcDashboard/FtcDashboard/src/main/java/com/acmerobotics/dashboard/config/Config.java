package com.acmerobotics.dashboard.config;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotation that specifies configuration classes.
 *
 * <p>All public, static, non-final fields of the class will be automatically added as configuration
 * variables in the dashboard. When the user saves new values, these fields are correspondingly
 * updated. Classes annotated with {@link com.qualcomm.robotcore.eventloop.opmode.Disabled} are
 * ignored.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Config {
    /**
     * Name of this block of configuration variables. Defaults to the class's simple name.
     */
    String value() default "";
}
