package org.firstinspires.ftc.teamcode.robotSubSystems.Camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    private static OpenCvWebcam webcam1 = null;
    public static OpenCvCamera camera;

    public static void init(HardwareMap hardwareMap) {
        //Initiates the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Camera name from init in the Driver Station
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Camera monitor view
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Creates an object to show the camera view in the Telemetry area
        //Sets the pipeline

        camera.setPipeline(new TestPipeline());
        //Opening the camera
        OpenCvCamera finalCamera = camera;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //live streaming
            public void onOpened() {
                finalCamera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //it will be run if the camera could not be opened
            }
        });
    }


    public static void runDetection(HardwareMap hardwareMap) {
        WebcamName webcamName =  hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new TestPipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}
