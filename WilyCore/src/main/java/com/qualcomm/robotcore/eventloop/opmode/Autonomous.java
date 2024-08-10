package com.qualcomm.robotcore.eventloop.opmode;


import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Documented
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Autonomous {
    /**
     * The name to be used on the driver station display. If empty, the name of
     * the OpMode class will be used.
     * @return the name to use for the OpMode in the driver station.
     */
    String name() default "";

    /**
     * Optionally indicates a group of other OpModes with which the annotated
     * OpMode should be sorted on the driver station OpMode list.
     * @return the group into which the annotated OpMode is to be categorized
     */
    String group() default "";

    /**
     * The name of the TeleOp OpMode you'd like to have automagically preselected
     * on the Driver Station when selecting this Autonomous OpMode. If empty, then
     * nothing will be automagically preselected.
     *
     * @return see above
     */
    String preselectTeleOp() default "";

}
