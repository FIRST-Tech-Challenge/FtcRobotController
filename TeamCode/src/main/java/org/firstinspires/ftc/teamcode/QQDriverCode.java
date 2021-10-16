package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "QQ code")
public class QQDriverCode  extends LinearOpMode {
    double hue;//color we want, yellow
    OpenCvCamera cam;// webcam
    int width;
    int height;
    QQLeak mainPipeline;
    double sensitivity;

    @Override
    public void runOpMode() throws InterruptedException{
        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());//view for viewing what the webcam sees I think, on ds. Double check :grimacing:
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);

        cam.openCameraDevice();//start the webcam

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                mainPipeline = new QQLeak(telemetry);//create new pipeline
                cam.setPipeline(mainPipeline);//set webcam pipeline

                width = 640;
                height = 480;

                cam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);//can add rotation if needed
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error...",":(");
                System.exit(0);
            }
        });



        waitForStart();

        //now start button is pressed, robot go!

        int barcode = 0; //level 1 is bottom, level 2 is mid, 3 is up

        //idk how we're measuring the duckies yet but this is where we set target zones!
    }
}
