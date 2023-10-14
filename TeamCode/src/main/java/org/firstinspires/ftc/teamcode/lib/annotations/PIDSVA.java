package org.firstinspires.ftc.teamcode.lib.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Repeatable(Controllers.class)
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface PIDSVA {
    String name();
    double P();
    double I();
    double D();
    double S();
    double V();
    double A();
}
