package com.SCHSRobotics.HAL9001.system.robot;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation used to specify which camera or cameras a HALPipeline should be passed to.
 * <p>
 * Creation Date: 9/24/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see HALPipeline
 * @see VisionSubSystem
 * @see Robot
 * @see CameraManager
 * @see InternalCamera
 * @see ExternalCamera
 * @since 1.1.0
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Repeatable(Camera.CameraCollector.class)
public @interface Camera {

    /**
     * The id of the camera the annotation HAL Pipeline is linked to.
     *
     * @return The id of the camera the annotation HAL Pipeline is linked to.
     */
    String id();

    /**
     * An internal collector class used to allow the @Camera annotation to be repeated.
     *
     * @see Camera
     */
    @Documented
    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @interface CameraCollector {

        /**
         * The list of all used @Camera annotations.
         *
         * @return The list of all used @Camera annotations.
         *
         * @see Camera
         */
        Camera[] value();
    }
}