package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class BasicCamera {
    public OpenCvWebcam webcam;
    private LinearOpMode opMode;
    private HardwareMap hMap;
    private BasicPipeline pipeline;
    private Telemetry telemetry;
    public int color;

    public boolean cameraOpen = false;

    public BasicCamera (LinearOpMode o) {
        this.opMode = o;
        this.telemetry = opMode.telemetry;
        this.hMap = opMode.hardwareMap;
        int cameraMonitorViewId = this.hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.hMap.appContext.getPackageName());
        WebcamName webcamName = this.hMap.get(WebcamName.class, "WebcamMain");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new BasicPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                cameraOpen = true;
                telemetry.addLine("Camera has opened");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) { // this is not accurate
                telemetry.addLine("Camera Failed");
                telemetry.update();
            }
        });
    }

    public void update() {
        // whatever the camera needs to do will be written in these methods.
    }
}
