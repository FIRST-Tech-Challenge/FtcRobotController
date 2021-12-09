package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.pipelines.OpenCVPipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Map;

public class OpenCVProvider implements VisionProvider {
    private OpenCvCamera camera;
    private OpenCVPipeline pipeline;

    private static final String TELEMETRY_NAME = "OpenCV Vision Provider";

    @Override
    public void initializeVision(HardwareMap hardwareMap) {
        pipeline = new OpenCVPipeline();
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(() -> {
            camera.startStreaming(Constants.WEBCAM_WIDTH, Constants.WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        });
    }

    @Override
    public void shutdownVision() {
        camera.closeCameraDevice();
        camera.stopStreaming();
    }

    @Override
    public Position getPosition() {
        return pipeline.getLastPosition();
    }

    @Override
    public void reset() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public boolean canSendDashboardImage() {
        return true;
    }

    @Override
    public Mat getDashboardImage() {
        return pipeline.getDashboardImage();
    }

    @Override
    public void update() {

    }
}
