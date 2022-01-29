package com.SCHSRobotics.HAL9001.system.robot;

import org.jetbrains.annotations.NotNull;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvTracker;

/**
 * An abstract class used to store HAL computer vision pipelines. Almost a complete copy of OpenCvTracker from EasyOpenCV.
 * <p>
 * Creation Date: 9/24/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see OpenCvTracker
 * @see HALTrackerAPI
 * @see VisionSubSystem
 * @see Mat
 * @since 1.1.0
 */
//todo add stop functionality
public abstract class HALPipeline {
    //The local copy of the input image.
    private Mat mat = new Mat();
    //Whether this pipeline has been stopped.
    boolean markedAsStopped = false;

    /**
     * Whether the HALPipeline uses the viewport. This can change mid-program if needed.
     *
     * @return Whether the HALPipeline uses the viewport. This can change mid-program if needed.
     */
    public abstract boolean useViewport();

    /**
     * Runs the desired computer vision algorithm on the input image, then outputs an image to (optionally) display on the viewport.
     *
     * @param input The input image.
     * @return The image to display on the viewport.
     * @see Mat
     */
    public abstract Mat processFrame(Mat input);

    /**
     * An internal, non-abstract version of the processFrame function that copies the input image to the local mat object and runs process frame on that object.
     * Used so that the input image is not accidentally changed by the computer vision pipeline.
     *
     * @param input The input image.
     * @return The image to display on the viewport.
     * @see Mat
     */
    protected final Mat processFrameInternal(@NotNull Mat input) {
        input.copyTo(mat);
        return processFrame(mat);
    }

    /**
     * Stops this pipeline, removing it from the pipeline tracker permanently.
     */
    protected final void stop() {
        markedAsStopped = true;
    }
}