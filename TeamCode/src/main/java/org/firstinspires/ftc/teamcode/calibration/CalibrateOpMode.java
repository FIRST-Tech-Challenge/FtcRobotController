package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.*;
import org.openftc.easyopencv.*;

import java.util.ArrayList;

public class CalibrateOpMode extends LinearOpMode {
    public OpenCvCamera phoneCamL = null, phoneCamR = null;
    public singleCalib dL = new singleCalib(), dR = new singleCalib();

    @Override
    public void runOpMode() throws InterruptedException {
        // Start cameras and stuff
        phoneCamL = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        phoneCamR = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        phoneCamL.setPipeline(dL);
        phoneCamR.setPipeline(dR);

        phoneCamL.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {}

            //@Override
            public void onError(int errorCode) {}
        });

        phoneCamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {}

            //@Override
            public void onError(int errorCode) {}
        });

        // Gather camera data
        // Left
        ArrayList<Mat> imageL = new ArrayList<>();
        phoneCamL.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);



        phoneCamL.stopStreaming();




    }
}
