package org.firstinspires.ftc.teamcode.opMode.annotation;

import java.lang.annotation.*;

/**
 * Annotates a program. If the annotated type is located in the season package, it will be registered.
 * Make sure to include {@code enabled = true} if you wish to use the program.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Program {
    String name();
    boolean enabled() default false;
}