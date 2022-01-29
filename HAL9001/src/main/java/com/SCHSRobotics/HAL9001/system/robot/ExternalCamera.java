package com.SCHSRobotics.HAL9001.system.robot;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation meant to be used on OpenCVCamera objects or their subclasses in order to specify
 * that they are external cameras and to provide information about how to create them.
 * <p>
 * Creation Date: 12/24/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see InternalCamera
 * @see CameraManager
 * @see Robot
 * @since 1.1.0
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
@Repeatable(ExternalCamera.CameraContainer.class)
public @interface ExternalCamera {

    /**
     * The camera resolution width in pixels.
     *
     * @return The camera resolution width in pixels.
     */
    int resWidth();

    /**
     * The camera resolution height in pixels.
     *
     * @return The camera resolution height in pixels.
     */
    int resHeight();

    /**
     * The name of the camera in the (normal FTC) config system.
     *
     * @return The name of the camera in the (normal FTC) config system.
     */
    String configName();

    /**
     * The camera's unique identifier. If not set, defaults to using the camera's config name.
     *
     * @return The camera's unique identifier. If not set, defaults to using the camera's config name.
     */
    String uniqueId() default "";

    /**
     * Whether or not the camera uses the viewport to display output video.
     *
     * @return Whether or not the camera uses the viewport to display output video.
     */
    boolean usesViewport();

    /**
     * A container annotation used to contain all uses of @ExternalCamera so that the annotation can be repeated.
     *
     * @see ExternalCamera
     */
    @Documented
    @Retention(RetentionPolicy.RUNTIME)
    @Target({ElementType.FIELD})
    @interface CameraContainer {

        /**
         * An array containing all uses of @ExternalCamera so that the annotation can be repeated.
         *
         * @return An array containing all uses of @ExternalCamera.
         *
         * @see ExternalCamera
         */
        ExternalCamera[] value();
    }
}