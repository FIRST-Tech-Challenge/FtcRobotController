package com.wilyworks.simulator.framework;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

public class WilyOpenCvWebcam implements OpenCvWebcam {
    @Override
    public int openCameraDevice() {
        return 0;
    }

    @Override
    public void openCameraDeviceAsync(AsyncCameraOpenListener cameraOpenListener) {

    }

    @Override
    public void closeCameraDevice() {

    }

    @Override
    public void closeCameraDeviceAsync(AsyncCameraCloseListener cameraCloseListener) {

    }

    @Override
    public void showFpsMeterOnViewport(boolean show) {

    }

    @Override
    public void pauseViewport() {

    }

    @Override
    public void resumeViewport() {

    }

    @Override
    public void setViewportRenderingPolicy(ViewportRenderingPolicy policy) {

    }

    @Override
    public void setViewportRenderer(ViewportRenderer renderer) {

    }

    @Override
    public void startStreaming(int width, int height) {

    }

    @Override
    public void startStreaming(int width, int height, OpenCvCameraRotation rotation) {

    }

    @Override
    public void stopStreaming() {

    }

    @Override
    public void setPipeline(OpenCvPipeline pipeline) {

    }

    @Override
    public int getFrameCount() {
        return 0;
    }

    @Override
    public float getFps() {
        return 0;
    }

    @Override
    public int getPipelineTimeMs() {
        return 0;
    }

    @Override
    public int getOverheadTimeMs() {
        return 0;
    }

    @Override
    public int getTotalFrameTimeMs() {
        return 0;
    }

    @Override
    public int getCurrentPipelineMaxFps() {
        return 0;
    }

    @Override
    public void startRecordingPipeline(PipelineRecordingParameters parameters) {

    }

    @Override
    public void stopRecordingPipeline() {

    }

    @Override
    public void setMillisecondsPermissionTimeout(int ms) {

    }

    @Override
    public void startStreaming(int width, int height, OpenCvCameraRotation rotation, StreamFormat streamFormat) {

    }

    @Override
    public ExposureControl getExposureControl() {
        return null;
    }

    @Override
    public FocusControl getFocusControl() {
        return null;
    }

    @Override
    public PtzControl getPtzControl() {
        return null;
    }

    @Override
    public GainControl getGainControl() {
        return null;
    }

    @Override
    public WhiteBalanceControl getWhiteBalanceControl() {
        return null;
    }

    @Override
    public <T extends CameraControl> T getControl(Class<T> controlType) {
        return null;
    }

    @Override
    public CameraCalibrationIdentity getCalibrationIdentity() {
        return null;
    }
}
