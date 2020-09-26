package org.firstinspires.ftc.teamcode.support.hardware;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a setting on <code>Configurable</code> device as adjustable during runtime.
 * @see Configurable
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface Adjustable {

    /**
     * Minimum supported value for the setting being adjusted
     */
    double min();

    /**
     * Maximum supported value for the setting being adjusted
     */
    double max();

    /**
     * Smallest step to adjust the setting by
     */
    double step();
}
