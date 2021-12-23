package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.pipelines.OpenCVPipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;
import java.util.Map;

@Config
public class OpenCVProvider extends VisionProvider {
    private OpenCvCamera camera;
    private OpenCVPipeline pipeline;

    // Constants
    private static final String TELEMETRY_NAME = "OpenCV Vision Provider";
    public static int WEBCAM_WIDTH = 320;
    public static int WEBCAM_HEIGHT = 240;

    @Override
    public void initializeVision(HardwareMap hardwareMap) {
        pipeline = new OpenCVPipeline();
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void shutdownVision() {
        camera.stopStreaming();
        camera.closeCameraDevice();
    }

    @Override
    public Position getPosition() {
        return pipeline.getLastPosition();
    }

    @Override
    public void reset() {

    }

    @Override
    public boolean canSendDashboardImage() {
        return true;
    }

    @Override
    public Bitmap getDashboardImage() {
        return pipeline.getDashboardImage();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = super.getTelemetry(debug);

        if(debug) {
            telemetryMap.put("Frame Count", camera.getFrameCount());
            telemetryMap.put("FPS", String.format("%.2f", camera.getFps()));
            telemetryMap.put("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetryMap.put("Pipeline time ms", camera.getPipelineTimeMs());
            telemetryMap.put("Overhead time ms", camera.getOverheadTimeMs());
            telemetryMap.put("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void updateVision() {

    }
}
