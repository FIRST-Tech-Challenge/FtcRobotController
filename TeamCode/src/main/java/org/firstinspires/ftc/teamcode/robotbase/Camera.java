package org.firstinspires.ftc.teamcode.robotbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencvpipelines.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.function.BooleanSupplier;

public class Camera {
    private final OpenCvWebcam webcam;
//    protected RectangleTracking pipeline;
    protected AprilTagDetectionPipeline april_tag_pipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public Camera(HardwareMap hardwareMap, FtcDashboard dashboard, Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "webcam"), cameraMonitorViewId);
        dashboard.startCameraStream(webcam, 30);

        april_tag_pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        webcam.setPipeline(april_tag_pipeline);

        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

//    public RectangleTracking getPipeline(){
//        return pipeline;
//    }

    public AprilTagDetectionPipeline getPipeline(){
        return april_tag_pipeline;
    }

//    public double getObject_x(){
//        return pipeline.getElementsAnalogCoordinates()[0];
//    }
//
//    public double getObject_y(){
//        return pipeline.getElementsAnalogCoordinates()[1];
//    }
}
