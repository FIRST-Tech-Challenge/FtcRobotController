package org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_CENTERSTAGE.ColorDetectionPipeline;
import org.firstinspires.ftc.team6220_CENTERSTAGE.Constants;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

abstract public class InitializeCameras extends LinearOpMode {

    ColorDetectionPipeline pipeline1;
    int cameraMonitorViewId;

    public void initialize(Scalar minRange, Scalar maxRange) {

        OpenCvCamera robotCamera;

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        pipeline1 = new ColorDetectionPipeline();
        robotCamera.setPipeline(pipeline1);

        pipeline1.setRanges(minRange, maxRange);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robotCamera.startStreaming(Constants.CAMERA_WIDTH, Constants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }
        });

    }

}
