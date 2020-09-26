package org.firstinspires.ftc.teamcode.support.diagnostics;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks method as menu entry point for diagnostic TeleOps.
 * Annotated method must be public and must have either one or two
 *  (if two gamepads are supported) arguments of <code>EventManager</code> type.
 */
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface MenuEntry {

    /**
     * Menu label for this entry
     */
    String label();

    /**
     * (Optional) Menu group for this entry.
     * If specified, all menu entries with the same group will appear under the group name.
     * All menu entries are sorted by alphabetically by group name first (with entries without
     *  group specified being placed last) followed by entry label.
     */
    String group() default "";
}
