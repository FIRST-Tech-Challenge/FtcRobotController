package org.firstinspires.ftc.teamcode.vision.newrobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.util.myOpenCvCamera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class barcodeDetector {

    private myOpenCvCamera camera;
    private final Telemetry telemetry;
    private final boolean isUsingWebcam;
    private final String webcamName;
    private final HardwareMap hardwareMap;
    private barcodePipeline ftclibPipeline;

    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;

    // The constructor is overloaded to allow the use of webcam instead of the phone camera
    public barcodeDetector(Telemetry telemetry, HardwareMap hMap, String webcamName) {
        this.telemetry = telemetry;
        hardwareMap = hMap;
        isUsingWebcam = true;
        this.webcamName = webcamName;
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be usingf
        if (isUsingWebcam) {
            int cameraMonitorViewId = hardwareMap
                    .appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = (myOpenCvCamera) OpenCvCameraFactory.getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        } else {
            int cameraMonitorViewId = hardwareMap
                    .appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = (myOpenCvCamera) OpenCvCameraFactory.getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        //Set the pipeline the camera should use and start streaming
        camera.setPipeline(ftclibPipeline = new barcodePipeline(telemetry));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public void setRectangles(double topRectWidthPercentage, double topRectHeightPercentage,double middleRectWidthPercentage,double middleRectHeightPercentage,double bottomRectWidthPercentage,double bottomRectHeightPercentage) {
        ftclibPipeline.setRectangles(topRectWidthPercentage,topRectHeightPercentage,middleRectWidthPercentage,middleRectHeightPercentage,bottomRectWidthPercentage,bottomRectHeightPercentage);
    }
}
