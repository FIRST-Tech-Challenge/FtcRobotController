package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Freight")
public class AutonomousCompetition extends MasterAutonomous {
    int allianceSide = 1;
    int barcodeIndex = 0;
    boolean farSide = false;
    @Override
    public void runOpMode() throws InterruptedException {
        // set up detection
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
            telemetry.addLine("False for starting on side nearest warehouse");
            telemetry.addData("Starting position", farSide);
            telemetry.addData("index", barcodeIndex);
            telemetry.update();
            idle();
        }

        waitForStart();

        // raise arm to position depending on barcode
        if (barcodeIndex == 0) {
            // highest level
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_3, 0.5);

        } else if (barcodeIndex == 1) {
            // middle level
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_2, 0.5);

        } else {
            // lowest level
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_1, 0.5);
        }

        // ----------------- MOVING -----------------
        if (!farSide) {
            moveInches(2, 0.7);
            pivot(-10 * allianceSide, 0.7);
            moveInches(4, 0.7);
            pivot(-90 * allianceSide, 0.7);
            moveInches(22, 0.4);
            pivot(0, 0.7);
            if (barcodeIndex == 0) {
                moveInches(4, 0.7);
            }
            moveInches(19, 0.4);
            // drop element here
            dropElement();
            //sleep(5000);
            moveInches(-10, 0.7);
            pivot(90 * allianceSide, 0.7);
            // back up into parking zone here
            moveInches(70, 1.0);
        } else {
            moveInches(6, 0.7);
            pivot(90 * allianceSide, 0.7);
            moveInches(20, 0.4);
            pivot(0, 0.7);
            moveInches(2, 0.4);
            // drop element
            dropElement();
            moveInches(-5, 0.7);
            pivot(90 * allianceSide, 0.7);
            // back up into parking zone

        }


    }

    public void dropElement() {
        grabberServo.setPosition(GRABBER_OUT);
    }
}
