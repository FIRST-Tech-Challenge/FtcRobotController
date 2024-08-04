package org.firstinspires.ftc.teamcode.hardware;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Specifies that a motor should start with its direction reversed.
 * The annotated field must be a DcMotorSimple or subclass.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
@Documented
public @interface Reversed {}
