package org.firstinspires.ftc.team417_2021;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Test Detection ")
public class TestTeamElementOpenCVDetection extends LinearOpMode {

    OpenCvCamera webcam;
    BarcodeDetectionOpenCV barcodeDetector = new BarcodeDetectionOpenCV();

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Side Webcam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        webcam.setPipeline(barcodeDetector);
        telemetry.addLine("Ready for Start");
        waitForStart();

        while (opModeIsActive()) {
            String side = "";
            if (barcodeDetector.barcodeIndex == 0){
                side = "left";
            }
            else if (barcodeDetector.barcodeIndex == 1){
                side = "middle";
            }
            else if (barcodeDetector.barcodeIndex == 2){
                side = "right";
            }

            telemetry.addData("Side", side);
            telemetry.update();
        }
    }
}
