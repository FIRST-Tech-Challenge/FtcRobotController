package com.SCHSRobotics.HAL9001.system.robot;

import android.util.Log;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * A static class used for managing internal and external cameras.
 * <p>
 * Creation Date: 9/24/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see InternalCamera
 * @see ExternalCamera
 * @see CameraType
 * @see Robot
 * @see VisionSubSystem
 * @see HALPipeline
 * @see HALTrackerAPI
 * @since 1.1.0
 */
public final class CameraManager {
    //The logging tag for this class. Used to print non-crash-worthy errors using Log.e.
    private static final String LOGGING_TAG = "HAL Camera Manager";
    //A map relating camera ids to OpenCVCamera objects (external cameras).
    private static final Map<String, OpenCvCamera> externalCameras = new HashMap<>();
    //A map relating camera ids to camera resolutions.
    private static final Map<String, Size> resolutionMap = new HashMap<>();
    //A map relating camera ids to whether that camera has been started.
    private static final Map<String, Boolean> isStartedMap = new HashMap<>();
    //A map relating camera ids to the HALPipeline trackers associated with each camera.
    private static final Map<String, HALTrackerAPI> trackerAPIMap = new HashMap<>();
    //The internal camera, if present.
    private static OpenCvCamera internalCamera;
    //The internal camera id, if present.
    private static String internalCameraId;

    /**
     * Private constructor for Camera Manager to make it a static class.
     */
    private CameraManager() {
    }

    /**
     * Adds a camera to the camera manager.
     *
     * @param id         The id of the camera.
     * @param camera     The camera object to add.
     * @param cameraType The type of camera being added (INTERNAL or EXTERNAL).
     * @param resolution The resolution of the camera.
     * @see OpenCvCamera
     * @see CameraType
     * @see Size
     */
    protected static void addCamera(String id, OpenCvCamera camera, @NotNull CameraType cameraType, Size resolution) {
        resolutionMap.put(id, resolution);
        isStartedMap.put(id, false);
        trackerAPIMap.put(id, new HALTrackerAPI());

        switch (cameraType) {
            case INTERNAL:
                internalCameraId = id;
                internalCamera = camera;
                break;
            case EXTERNAL:
                externalCameras.put(id, camera);
                break;
        }
    }

    /**
     * Adds the pipeline to all cameras.
     *
     * @param pipeline The pipeline to add.
     * @see HALPipeline
     * @see HALTrackerAPI
     */
    protected static void addPipelineToAll(HALPipeline pipeline) {
        for (String cameraId : trackerAPIMap.keySet()) addPipeline(cameraId, pipeline);
    }

    /**
     * Adds a pipeline to a camera with the given id.
     *
     * @param cameraId The id of the camera the pipeline is being added to.
     * @param pipeline The pipeline to add.
     * @see HALPipeline
     * @see HALTrackerAPI
     */
    protected static void addPipeline(String cameraId, HALPipeline pipeline) {
        if (!cameraExists(cameraId)) {
            Log.e(LOGGING_TAG, "Camera " + cameraId + " does not exist.");
            return;
        }

        Objects.requireNonNull(trackerAPIMap.get(cameraId)).addPipeline(pipeline);
    }

    /**
     * Removes a pipeline to a camera with the given id.
     *
     * @param cameraId The id of the camera the pipeline is being removed from.
     * @param pipeline The pipeline to remove.
     * @see HALPipeline
     * @see HALTrackerAPI
     */
    protected static void removePipeline(String cameraId, HALPipeline pipeline) {
        if (cameraExists(cameraId)) {
            Objects.requireNonNull(trackerAPIMap.get(cameraId)).removePipeline(pipeline);
        }
    }

    /**
     * Gets whether a camera with the given id exists.
     *
     * @param cameraId The id of the camera to look for.
     * @return Whether a camera with the given id exists.
     */
    protected static boolean cameraExists(@NotNull String cameraId) {
        return cameraId.equals(internalCameraId) || externalCameras.containsKey(cameraId);
    }

    /**
     * Runs all pipelines for all cameras.
     *
     * @see OpenCvCamera
     * @see HALPipeline
     * @see HALTrackerAPI
     */
    protected static void runPipelines() {
        for (String cameraId : trackerAPIMap.keySet()) {
            runHALTrackerAPI(cameraId, Objects.requireNonNull(trackerAPIMap.get(cameraId)));
        }
    }

    /**
     * Runs all pipelines for a camera with the given id.
     *
     * @param cameraId      The id of the camera that will be running the pipelines.
     * @param halTrackerAPI The collection of pipelines to run.
     * @see OpenCvCamera
     * @see HALPipeline
     * @see HALTrackerAPI
     */
    private static void runHALTrackerAPI(String cameraId, HALTrackerAPI halTrackerAPI) {
        if (!cameraExists(cameraId)) {
            Log.e(LOGGING_TAG, "Camera " + cameraId + " does not exist.");
            return;
        }
        boolean isInternalCamera = cameraId.equals(internalCameraId);

        OpenCvCamera camera;
        if (isInternalCamera) camera = internalCamera;
        else camera = Objects.requireNonNull(externalCameras.get(cameraId));

        if (!Objects.requireNonNull(isStartedMap.get(cameraId))) {
            camera.setPipeline(halTrackerAPI);
            Size resolution = Objects.requireNonNull(resolutionMap.get(cameraId));

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming((int) resolution.width, (int) resolution.height, OpenCvCameraRotation.UPRIGHT);
                    if(isInternalCamera) camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                    camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                    // Usually this is where you'll want to start streaming from the camera (see section 4)
                }
                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
            isStartedMap.put(cameraId, true);
        }
    }

    /**
     * Reset the camera manager.
     */
    protected static void resetManager() {
        externalCameras.clear();
        resolutionMap.clear();
        isStartedMap.clear();
        trackerAPIMap.clear();
        internalCamera = null;
        internalCameraId = null;
    }

    /**
     * Stops the internal camera.
     *
     * @see OpenCvCamera
     */
    protected static void stopInternalCamera() {
        if (internalCamera != null) {
            internalCamera.stopStreaming();
            internalCamera.closeCameraDevice();
        }
    }

    /**
     * Overrides the internal camera with a new internal camera object. Used to switch from front mode to back mode.
     *
     * @param newInternalCamera The new internal camera object.
     * @see OpenCvCamera
     * @see Robot
     */
    protected static void overrideInternalCamera(OpenCvCamera newInternalCamera) {
        if (internalCamera != null) {
            internalCamera = newInternalCamera;
            isStartedMap.put(internalCameraId, false);
            runHALTrackerAPI(internalCameraId, trackerAPIMap.get(internalCameraId));
        } else {
            Log.e(LOGGING_TAG, "Tried to override internal camera, but there is no defined internal camera.");
        }
    }

    /**
     * Gets a camera object with the specified id.
     *
     * @param cameraId The id of the camera to get.
     * @param <T>      The specific subtype of camera to return.
     * @return The camera object associated with the given id.
     * @see OpenCvCamera
     * @see Robot
     */
    @SuppressWarnings("unchecked")
    @Nullable
    protected static <T extends OpenCvCamera> T getCamera(String cameraId) {
        if (internalCamera != null && cameraId.equals(internalCameraId)) return (T) internalCamera;
        else if (externalCameras.containsKey(cameraId)) return (T) externalCameras.get(cameraId);
        else {
            Log.e(LOGGING_TAG, "Tried to find camera with id " + cameraId + " but no camera with that id was registered.");
            return null;
        }
    }
}