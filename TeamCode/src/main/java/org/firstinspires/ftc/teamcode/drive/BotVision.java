package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class BotVision {
    public OpenCvWebcam webcam;
    FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry tele = dash.getTelemetry();
    public OpenCvPipeline pipeline;
    public boolean inited = false;
    public boolean opened = false;
    OpenCvPipeline p = null;


    public void init(HardwareMap hardwareMap, OpenCvPipeline pipeline) {
        this.pipeline = pipeline;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(this.pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                tele.addLine("Opened!");
                tele.update();
                opened = true;
            }

            @Override
            public void onError(int errorCode) {
                tele.addData("Crashed", "camera");
                tele.update();
            }
        });
        inited = true;
    }
    public void init(HardwareMap hardwareMap, OpenCvPipeline pipeline, String webcamName) {
        this.pipeline = pipeline;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        webcam.setPipeline(this.pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                tele.addLine("Opened!");
                tele.update();
            }

            @Override
            public void onError(int errorCode) {
                tele.addData("Crashed", "camera");
                tele.update();
            }
        });
        inited = true;
    }


}