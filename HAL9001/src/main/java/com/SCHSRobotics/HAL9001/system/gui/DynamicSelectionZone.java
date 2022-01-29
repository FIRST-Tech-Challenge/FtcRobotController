package com.SCHSRobotics.HAL9001.system.gui;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation specifying that the selection zone for a given HAL Menu can expand when new lines are added.
 * When the selection zone expands, the specified pattern will be used to fill the new rows.
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see HALMenu
 * @see SelectionZone
 * @since 1.1.0
 */
@Documented
@Inherited
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface DynamicSelectionZone {

    /**
     * The selection zone pattern used to fill new rows of the menu.
     *
     * @return The selection zone pattern used to fill new rows of the menu.
     * @see HALMenu
     * @see SelectionZone
     */
    boolean[] pattern();
}