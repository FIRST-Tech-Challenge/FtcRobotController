package com.technototes.logger;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import static java.lang.annotation.ElementType.FIELD;
import static java.lang.annotation.ElementType.LOCAL_VARIABLE;
import static java.lang.annotation.ElementType.METHOD;

@Repeatable(Log.Logs.class)
@Retention(RetentionPolicy.RUNTIME)
@Target(value={FIELD, LOCAL_VARIABLE, METHOD})
public @interface Log {
    enum thingy{
     EEE
    }
    thingy thing() default thingy.EEE;
    int index() default -1;

    String name() default "Log";

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD, ElementType.METHOD})
    @interface Logs {
        Log[] value();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface Number{
        int index() default -1;

        String name() default "Number";
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface NumberBar{
        int index() default -1;

        double min() default -1;
        double max() default 1;
        double scale() default 0.1;

        String name() default "NumberBar";
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface NumberSlider{
        int index() default -1;

        double min() default -1;
        double max() default 1;
        double scale() default 0.1;

        String name() default "NumberSlider";
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface Boolean{
        int index() default -1;

        String valueWhenTrue() default "true";
        String valueWhenFalse() default "false";

        String name() default "Boolean";
    }
}
