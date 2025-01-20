package org.firstinspires.ftc.teamcode.Camera.Parents;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


public class Webcam extends LinearOpMode {
    protected OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        initCam(telemetry);

        camOpen();

        telemetry.addLine("Waiting for start");

        telemetry.update();

        waitForStart();

        camClose();

    }
    public void initCam(Telemetry telemetry){

    }
    public void camOpen() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    public void camClose(){
        webcam.stopStreaming();
    }
}
