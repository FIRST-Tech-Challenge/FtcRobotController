package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Comp")
public class AutonomousCompetition extends MasterAutonomous {
    int allianceSide = 1;
    int barcodeIndex = 0;
    boolean farSide = false;
    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvCamera webcam;
        BarcodeDetectionOpenCV barcodeDetector = new BarcodeDetectionOpenCV();
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
        // todo uncomment when webcam is on robot

        initializeHardware();
        telemetry.addLine("Ready to Start");
        telemetry.update();

        while (!opModeIsActive()) {
            if (gamepad1.a) {
                allianceSide *= -1;
            }
            barcodeIndex = barcodeDetector.index;
            telemetry.addLine("1 for blue, -1 for red");
            telemetry.addData("Alliance", allianceSide);
            telemetry.addData("index", barcodeIndex);
            // todo uncomment when webcam is on robot
            telemetry.update();
            idle();
        }

        waitForStart();
        if (!farSide) {
            move3(6, 0.7);
            pivot(-30 * allianceSide, 0.7);
            move3(20, 0.4);
            // drop element here
            dropElement();
            move3(-5, 0.7);
            pivot(-90 * allianceSide, 0.7);
            // back up into parking zone here
        } else {
            move3(6, 0.7);
            pivot(30 * allianceSide, 0.7);
            move3(20, 0.4);
            // drop element
            dropElement();
            move3(-5, 0.7);
            pivot(-90 * allianceSide, 0.7);
            // back up into parking zone

        }


    }

    public void dropElement() {
        if (barcodeIndex == 0) {
            // highest level
        } else if (barcodeIndex == 1) {
            // middle level
        } else {
            // lowest level
        }
    }
}
