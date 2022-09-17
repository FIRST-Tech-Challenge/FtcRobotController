/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.openftc.easyopencv;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

public interface OpenCvCamera extends CameraStreamSource
{
    public static final int CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE = -1;
    public static final int CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE = -2;

    /***
     * Open the connection to the camera device. If the camera is
     * already open, this will not do anything.
     *
     * You must call this before calling:
     * {@link #startStreaming(int, int)}
     * or {@link #startStreaming(int, int, OpenCvCameraRotation)}
     * or {@link #stopStreaming()}
     *
     * See {@link #closeCameraDevice()}
     */
    @Deprecated
    int openCameraDevice();

    /***
     * Performs the same thing as {@link #openCameraDevice()} except
     * in a non-blocking fashion, with a callback delivered to you
     * when the operation is complete. This can be particularly helpful
     * if using a webcam, as opening/starting streaming on a webcam can
     * be very expensive time-wise.
     *
     * It is reccommended to start streaming from your listener:
     *
     *      camera = OpenCvCameraFactory.create..............
     *      camera.setPipeline(new SomePipeline());
     *
     *      camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
     *      {
     *          @Override
     *          public void onOpened()
     *          {
     *              camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
     *          }
     *      });
     *
     * NOTE: the operation performed in the background thread is synchronized
     * with the main lock, so any calls to camera.XYZ() will block until the
     * callback has been completed.
     *
     * @param cameraOpenListener the listener to which a callback will be
     *                           delivered when the camera has been opened
     */
    void openCameraDeviceAsync(AsyncCameraOpenListener cameraOpenListener);

    interface AsyncCameraOpenListener
    {
        /**
         * Called if the camera was successfully opened
         */
        void onOpened();

        /**
         * Called if there was an error opening the camera
         * @param errorCode reason for failure
         */
        void onError(int errorCode);
    }

    /***
     * Close the connection to the camera device. If the camera is
     * already closed, this will not do anything.
     */
    void closeCameraDevice();

    /***
     * Performs the same this as {@link #closeCameraDevice()} except
     * in a non-blocking fashion.
     *
     * NOTE: the operation performed in the background thread is synchronized
     * with the main lock, so any calls to camera.XYZ() will block until the
     * callback has been completed.
     *
     * @param cameraCloseListener the listener to which a callback will be
     *                            delivered when the camera has been closed
     */
    void closeCameraDeviceAsync(AsyncCameraCloseListener cameraCloseListener);

    interface AsyncCameraCloseListener
    {
        void onClose();
    }

    /***
     * If a viewport container ID was passed to the constructor of
     * the implementing class, this method controls whether or not
     * to show some info/statistics on top of the camera feed.
     *
     * @param show whether to show some info on top of the camera feed
     */
    void showFpsMeterOnViewport(boolean show);

    /***
     * If a viewport container ID was passed to the constructor of
     * the implementing class, this method will "pause" the viewport
     * rendering thread. This can reduce CPU, memory, and power load.
     * For instance, this could be useful if you wish to see the live
     * camera preview as you are initializing your robot, but you no
     * longer require the live preview after you have finished your
     * initialization process. See {@link #resumeViewport()}
     */
    void pauseViewport();

    /***
     * If a viewport container ID was passed to the constructor of
     * the implementing class, and the viewport was previously paused
     * by {@link #pauseViewport()}, this method will "unpause" the
     * viewport rendering thread, so that you can see the live camera
     * feed on the screen again.
     */
    void resumeViewport();

    /***
     * The way the viewport will render the live preview
     *
     * IMPORTANT NOTE: The policy you choose here has NO IMPACT on the
     * frames passed to your pipeline. This ONLY affects how the frames
     * you return from your pipeline are rendered to the viewport.
     */
    enum ViewportRenderingPolicy
    {
        /*
         * This policy will minimize the CPU load caused by the viewport
         * rendering, at the expense of displaying a preview which is 90
         * or 180 out from what you might expect in some orientations.
         * (Note: unlike when viewing a still picture which is taken sideways,
         * simply rotating the phone physically does not correct the view
         * because when doing so you also rotate the camera on the phone).
         */
        MAXIMIZE_EFFICIENCY,

        /*
         * This policy will ensure that the live view in the viewport is
         * always displayed in a logical orientation, at the expense of
         * additional CPU load.
         */
        OPTIMIZE_VIEW
    }

    /***
     * Set the viewport rendering policy for this camera
     *
     * @param policy see {@link ViewportRenderingPolicy}
     */
    void setViewportRenderingPolicy(ViewportRenderingPolicy policy);

    /***
     * The renderer the viewport will use to render the live preview
     * NOTE: this is different than {@link ViewportRenderingPolicy}.
     * The rendering policy controls how the preview will look, but
     * this controls how the rendering is *actually done*
     */
    enum ViewportRenderer
    {
        /**
         * Default, if not otherwise specified. Historically this was the only option
         * (Well, technically there wasn't an option for this at all before, but you get the idea)
         */
        SOFTWARE,

        /**
         * Can provide a much smoother live preview at higher resolutions, especially if
         * you're using {@link ViewportRenderingPolicy#OPTIMIZE_VIEW}.
         * However, using GPU acceleration has been observed to occasionally cause crashes
         * in libgles.so / libutils.so on some devices, if the activity orientation is changed
         * (i.e. you rotate the device) while a streaming session is in flight. Caveat emptor.
         */
        GPU_ACCELERATED
    }

    /***
     * Set the viewport renderer for this camera
     * NOTE: This may ONLY be called if there is not currently a streaming session in
     * flight for this camera.
     *
     * @param renderer see {@link ViewportRenderer}
     * @throws IllegalStateException if called while a streaming session is in flight
     */
    void setViewportRenderer(ViewportRenderer renderer);

    /***
     * Tell the camera to start streaming images to us! Note that you must make sure
     * the resolution you specify is supported by the camera. If it is not, an exception
     * will be thrown.
     *
     * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
     * supports streaming from the webcam in the uncompressed YUV image format. This means
     * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
     * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
     *
     * Also see the alternate {@link #startStreaming(int, int, OpenCvCameraRotation)} method.
     *
     * @param width the width of the resolution in which you would like the camera to stream
     * @param height the height of the resolution in which you would like the camera to stream
     */
    void startStreaming(int width, int height);

    /***
     * Same as {@link #startStreaming(int, int)} except for:
     *
     * @param rotation the rotation that the camera is being used in. This is so that
     *                 the image from the camera sensor can be rotated such that it is always
     *                 displayed with the image upright. For a front facing camera, rotation is
     *                 defined assuming the user is looking at the screen. For a rear facing camera
     *                 or a webcam, rotation is defined assuming the camera is facing away from the user.
     */
    void startStreaming(int width, int height, OpenCvCameraRotation rotation);

    /***
     * Stops streaming images from the camera (and, by extension, stops invoking your vision
     * pipeline), without closing ({@link #closeCameraDevice()}) the connection to the camera.
     */
    void stopStreaming();

    /***
     * Specify the image processing pipeline that you wish to be invoked upon receipt
     * of each frame from the camera. Note that switching pipelines on-the-fly (while
     * a streaming session is in flight) *IS* supported.
     *
     * @param pipeline the image processing pipeline that you wish to be invoked upon
     *                 receipt of each frame from the camera.
     */
    void setPipeline(OpenCvPipeline pipeline);

    /***
     * Get the number of frames that have been received from the camera and processed by
     * your pipeline since {@link #startStreaming(int, int)} was called.
     *
     * @return the number of frames that have been received from the camera and processed
     *         by your pipeline since {@link #startStreaming(int, int)} was called.
     */
    int getFrameCount();

    /***
     * Get the current frame rate of the overall system (including your pipeline as well as
     * overhead) averaged over the last 30 frames.
     *
     * @return the current frame rate of the overall system (including your pipeline as well
     *         as overhead) averaged over the last 30 frames.
     */
    float getFps();

    /***
     * Get the current execution time (in milliseconds) of your pipeline, averaged over the
     * last 30 frames.
     *
     * @return the current execution time (in milliseconds) of your pipeline, averaged
     *         over the last 30 frames.
     */
    int getPipelineTimeMs();

    /***
     * Get the current system overhead time (in milliseconds) for each frame, averaged over
     * the last 30 frames.
     *
     * @return the current system overhead time (in milliseconds) for each frame, averaged
     * over the last 30 frames
     */
    int getOverheadTimeMs();

    /***
     * Get the current total processing time (in milliseconds) for each frame (including
     * pipeline and overhead), averaged over the last 30 frames.
     *
     * @return the current total processing time (in milliseconds) for each frame (including
     *         pipeline and overhead), averaged over the last 30 frames.
     */
    int getTotalFrameTimeMs();

    /***
     * Get the current theoretically maximum frame rate that your pipeline (and overhead)
     * could achieve. This is useful for identifying whether or not your pipeline is the
     * bottleneck in the system. For instance, if {@link #getFps()} reports that the system
     * is running at 10FPS, and this method reported that your theoretical maximum FPS is
     * 12, then your pipeline is the bottleneck. Conversely, if {@link #getFps()} reported that
     * the system was running at 25FPS, and this method reported that your theoretical maximum
     * FPS is 100, then the camera would be the bottleneck.
     *
     * @return the current theoretically maximum frame rate that your pipeline (and overhead)
     *         could achieve.
     */
    int getCurrentPipelineMaxFps();

    /***
     * Start recording the output of the camera's current pipeline
     * (If no pipeline is set, then the plain camera image is recorded)
     * A streaming session must be in flight before this can be called.
     * The recording will be automatically stopped when the streaming
     * session is stopped (whether that be manually or automatically at
     * the end of the OpMode), but can also be stopped independently by
     * calling {@link #stopRecordingPipeline()}
     *
     * @param parameters the parameters which define how the recording should done
     * @throws IllegalStateException if called before streaming is started
     * @throws IllegalStateException if recording was started previously
     */
    void startRecordingPipeline(PipelineRecordingParameters parameters);

    /***
     * Stops recording the output of the camera's current pipeline,
     * if a recording session is currently active.
     */
    void stopRecordingPipeline();
}
