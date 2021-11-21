package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomous Competition", group = "Autonomous")
public class AutonomousCompetition extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new BarcodeDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        int barcode = BarcodeDetectionPipeline.barcode;

        pauseMillis(500);

        if (BarcodeDetectionPipeline.barcode != barcode) {
            barcode = BarcodeDetectionPipeline.barcode;
        }

        telemetry.addData("Barcode: ", barcode);
        telemetry.update();

        pauseMillis(15000);

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        switch (barcode) {
            case 1:
                // stuff here
                break;

            case 2:
                // different stuff here
                break;

            case 3:
                // more different stuff here
                break;
        }
    }
}