package com.wilyworks.simulator.framework;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

public class WilyVisionPortal extends VisionPortal {
    public WilyVisionPortal(CameraName camera, int cameraMonitorViewId, boolean autoPauseCameraMonitor, Size cameraResolution, VisionPortal.StreamFormat webcamStreamFormat, VisionProcessor[] processors)
    {
        for (VisionProcessor processor: processors) {
            if ((processor instanceof WilyAprilTagProcessor) &&
                    (camera instanceof WilyWebcam)) {
                WilyAprilTagProcessor wilyAprilTagProcessor = (WilyAprilTagProcessor) processor;
                WilyWebcam wilyWebCam = (WilyWebcam) camera;
                wilyAprilTagProcessor.initialize(((WilyWebcam) camera).wilyCamera);
            }
        }
    }

    @Override
    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {

    }

    @Override
    public boolean getProcessorEnabled(VisionProcessor processor) {
        return false;
    }

    @Override
    public VisionPortal.CameraState getCameraState() {
        return null;
    }

    @Override
    public void saveNextFrameRaw(String filename) {

    }

    @Override
    public void stopStreaming() {

    }

    @Override
    public void resumeStreaming() {
    }

    @Override
    public void stopLiveView() {
    }

    @Override
    public void resumeLiveView() {
    }

    @Override
    public float getFps() {
        return 0;
    }

    @Override
    public <T extends CameraControl> T getCameraControl(Class<T> controlType) {
        return null;
    }

    @Override
    public void setActiveCamera(WebcamName webcamName) {
    }

    @Override
    public WebcamName getActiveCamera() {
        return null;
    }

    @Override
    public void close() {
    }
}
