package com.SCHSRobotics.HAL9001.system.robot;

import org.openftc.easyopencv.OpenCvInternalCamera;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation meant to be used on OpenCVCamera objects or their subclasses in order to specify
 * that they are external cameras and to provide information about how to create them.
 * <p>
 * Creation Date: 9/24/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see ExternalCamera
 * @see CameraManager
 * @see Robot
 * @since 1.1.0
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface InternalCamera {

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
     * The internal camera direction (FRONT or BACK).
     *
     * @return The internal camera direction (FRONT or BACK).
     */
    OpenCvInternalCamera.CameraDirection direction() default OpenCvInternalCamera.CameraDirection.BACK;

    /**
     * Whether or not the camera uses the viewport to display output video.
     *
     * @return Whether or not the camera uses the viewport to display output video.
     */
    boolean usesViewport();
}