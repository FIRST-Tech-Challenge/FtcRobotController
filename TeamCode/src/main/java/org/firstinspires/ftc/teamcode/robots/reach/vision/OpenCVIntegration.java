package org.firstinspires.ftc.teamcode.robots.reach.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.reach.utils.Constants;
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
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(Constants.WEBCAM_WIDTH, Constants.WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
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

    @Override
    public void reset() {}
}
