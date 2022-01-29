package com.SCHSRobotics.HAL9001.system.robot;

import org.jetbrains.annotations.NotNull;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * A modified version of EasyOpenCV's TrackerAPI used to manage HALPipelines.
 * <p>
 * Creation Date: 9/24/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see org.openftc.easyopencv.OpenCvTracker
 * @see HALPipeline
 * @see CameraManager
 * @see Robot
 * @since 1.1.0
 */
public final class HALTrackerAPI extends OpenCvPipeline {
    //A list of HALPipelines that do not use the viewport.
    private final List<HALPipeline> nonDisplayablePipelines = new ArrayList<>(), allPipelines = new ArrayList<>();
    //A queue of HALPipelines that do use the viewport. Only the one at the front of the queue is displayed.
    private final Queue<HALPipeline> displayablePipelines = new LinkedBlockingQueue<>();

    /**
     * Adds a pipeline to the tracker.
     *
     * @param pipeline The pipeline to add.
     * @see HALPipeline
     */
    public final synchronized void addPipeline(@NotNull HALPipeline pipeline) {
        if (pipeline.useViewport()) displayablePipelines.add(pipeline);
        else nonDisplayablePipelines.add(pipeline);
    }

    /**
     * Removes a pipeline from the tracker.
     *
     * @param tracker The pipeline to remove.
     *
     * @see HALPipeline
     */
    public final synchronized void removePipeline(HALPipeline tracker) {
        nonDisplayablePipelines.remove(tracker);
        displayablePipelines.remove(tracker);
        allPipelines.remove(tracker);
    }

    @Override
    public final synchronized Mat processFrame(Mat input) {
        //If no non-displayable or displayable pipelines are present, just return the input image.
        if (nonDisplayablePipelines.size() == 0 && displayablePipelines.size() == 0) return input;

        //Remove all pipelines that have been stopped.
        for (HALPipeline pipeline : allPipelines) {
            if (pipeline.markedAsStopped) removePipeline(pipeline);
        }

        //Go through all non-displayable pipelines. If one of them has changed to use the viewport, add it to the displayable pipelines queue.
        for (HALPipeline pipeline : new ArrayList<>(nonDisplayablePipelines)) {
            if (pipeline.useViewport()) {
                displayablePipelines.add(pipeline);
                nonDisplayablePipelines.remove(pipeline);
            }
        }

        //Go through all displayable pipelines. If one of them has changed to now use the viewport, add it to the non-displayable pipelines list.
        for (HALPipeline pipeline : new LinkedBlockingQueue<>(displayablePipelines)) {
            if (!pipeline.useViewport()) {
                displayablePipelines.remove(pipeline);
                nonDisplayablePipelines.add(pipeline);
            }
        }

        //If there are no displayable pipelines, run all non-displayable pipelines then return the input image.
        if (displayablePipelines.size() == 0) {
            for (HALPipeline pipeline : nonDisplayablePipelines)
                pipeline.processFrameInternal(input);
            return input;
        }

        //Run the current pipeline first and get the return image.
        HALPipeline currentPipeline = displayablePipelines.peek();
        Mat returnMat = currentPipeline.processFrameInternal(input);

        //Run all other displayable pipelines, then run the non-displayable pipelines.
        for (HALPipeline pipeline : displayablePipelines)
            if (!pipeline.equals(currentPipeline))
                pipeline.processFrameInternal(input);
        for (HALPipeline pipeline : nonDisplayablePipelines) pipeline.processFrameInternal(input);

        return returnMat;
    }

    @Override
    public final synchronized void onViewportTapped() {
        if (displayablePipelines.size() > 0) displayablePipelines.add(displayablePipelines.poll());
    }
}