package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.providers;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.pipelines.OpenCVPipeline;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;
import java.util.Map;

@Config
public class OpenCVProvider extends VisionProvider {
    private Bitmap noCameraBitmap;
    private OpenCvCamera camera;
    private OpenCVPipeline pipeline;
    private boolean cameraOpened;

    // Constants
    private static final String TELEMETRY_NAME = "OpenCV Vision Provider";
    public static int WEBCAM_WIDTH = 320;
    public static int WEBCAM_HEIGHT = 240;

    @Override
    public void initializeVision(HardwareMap hardwareMap) {
        pipeline = new OpenCVPipeline();
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        noCameraBitmap = Bitmap.createBitmap(320, 240, Bitmap.Config.RGB_565);
        Mat noCameraMat = new Mat(240, 320, CvType.CV_8UC3);
        Imgproc.putText(noCameraMat, "No Webcam Found", new Point(10, 120), Imgproc.FONT_HERSHEY_SIMPLEX,
                1, new Scalar(0, 0, 255), 3);
        Utils.matToBitmap(noCameraMat, noCameraBitmap);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
                cameraOpened = true;
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void shutdownVision() {
        if(cameraOpened) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    @Override
    public Position getPosition() {
        return cameraOpened ? pipeline.getLastPosition() : Position.HOLD;
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
        return cameraOpened ? pipeline.getDashboardImage() : noCameraBitmap;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = super.getTelemetry(debug);

        if(debug && cameraOpened) {
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
