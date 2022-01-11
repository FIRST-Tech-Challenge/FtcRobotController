package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.calib3d.Calib3d;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CalibOpMode extends LinearOpMode {
    OpenCvCamera l, r;
    singleCamCalibPipeline pl, pr;

    @Override
    public void runOpMode() throws InterruptedException {
        l = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        r = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        l.setPipeline(pl);
        r.setPipeline(pr);

        l.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
        pl.calibrate = true;
        while(!pl.isCalibrated) {

        }
        l.stopStreaming();

        r.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
        pr.calibrate = true;
        while(!pr.isCalibrated) {

        }
        r.stopStreaming();

        // Multi calib

        int flags = 0;
        flags |= Calib3d.CALIB_FIX_INTRINSIC;

        Calib3d.stereoCalibrate()
    }
}
