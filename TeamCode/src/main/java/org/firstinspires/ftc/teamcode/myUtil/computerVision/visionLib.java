package org.firstinspires.ftc.teamcode.myUtil.computerVision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class visionLib {
    OpenCvWebcam webcam1 = null;
    compVis test;

    public String init(LinearOpMode opMode, compVis.Colors color, Hardware r) {

            WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "webcam");
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            this.webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            test = new compVis(opMode, color,r);
            webcam1.setPipeline(test);
            webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {

                }
            });
            while (!opMode.isStarted());
            webcam1.closeCameraDevice();

            return test.loc;

    }
}
