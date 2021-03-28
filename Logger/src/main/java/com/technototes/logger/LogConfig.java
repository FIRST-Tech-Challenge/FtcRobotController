package com.technototes.logger;

import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import static java.lang.annotation.ElementType.FIELD;
import static java.lang.annotation.ElementType.LOCAL_VARIABLE;
import static java.lang.annotation.ElementType.METHOD;

/** Annotations for configuring Logs
 * @author Alex Stedman
 */
@Retention(RetentionPolicy.RUNTIME)

public @interface LogConfig {

    /** Annotation for determining when logged item will be sent to Telemetry
     *
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface Run{
        /** Run the log during the teleop Period
         *
         * @return The above condition
         */
        boolean duringRun() default true;
        /** Run the log during the init Period
         *
         * @return The above condition
         */
        boolean duringInit() default false;

    }

    /** Annotation for Whitelisting Opmodes to log this item
     *
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface Whitelist{
        /** The whitelisted opmodes
         *
         * @return Opmode Classes
         */
        Class<?>[] value();
    }
    /** Annotation for Blacklisting Opmodes to log this item
     *
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface Blacklist{
        /** The blacklisted opmodes
         *
         * @return Opmode Classes
         */
        Class<?>[] value();
    }

    /** Annotation to completely disable the entry
     *
     */
    @Retention(RetentionPolicy.RUNTIME)
    @Target(value={FIELD, LOCAL_VARIABLE, METHOD})
    @interface Disabled{

    }

}
