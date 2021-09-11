package org.firstinspires.ftc.teamcode.robots.goodBot.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.goodBot.utils.Constants;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCVIntegration implements VisionProvider {
    public OpenCvCamera camera;
    private boolean enableDashboard;
    public Pipeline pipeline;


    @Override
    public void initializeVision(HardwareMap hardwareMap, Viewpoint viewpoint, boolean enableDashboard) {
        pipeline = new Pipeline(enableDashboard);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(() -> {
                camera.startStreaming(Constants.WEBCAM_WIDTH, Constants.WEBCAM_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
        });
    }

    @Override
    public void shutdownVision() {
        camera.closeCameraDevice();
        camera.stopStreaming();
    }
    
    @Override
    public StackHeight detect() {
        return pipeline.getLastStackHeight();
    }

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public Mat process(Mat source0) {
        return null;
    }

    @Override
    public void reset() {}
}
