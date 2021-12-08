package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Freight")
public class FreightAuto extends MasterAutonomous {
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
        robot.setInitialAngle();


        // raise arm to position depending on barcode
        /*if (barcodeIndex == 0) {
            // highest level
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_3, 0.5);
        } else if (barcodeIndex == 1) {
            // middle level
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_2, 0.5);
        } else {
            // lowest level
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_1, 0.5);
        }*/

        // ----------------- RIGHT SIDE AUTO -----------------
        moveInches(10, 0.7);
        pivot(-90, 0.5);
        moveInches(21,0.7);
        pivot(-180, 0.5);
        moveInches(13, 0.5);
        carouselMotor.setPower(1);
        sleep(3200);
        carouselMotor.setPower(0);
        moveInches(-6,0.7);
        pivot(90,0.7);
        moveInches(48,0.7);
        pivot(0,0.7);


            wristServo.setPosition(0.0);
            if (barcodeIndex == 0) {
                runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_1, 0.5);
                runMotorToPosition(elbowMotor, ELBOW_LEVEL_1, 0.3);
                // 15 for level 3
                moveInches(12, 0.7);
                while(shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                    telemetry.addLine("pathli");
                }
            } else if (barcodeIndex == 1) {
                runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_2, 0.5);
                runMotorToPosition(elbowMotor, ELBOW_LEVEL_2, 0.3);
                // 15 for level 3
                moveInches(12, 0.7);
                while(shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                    telemetry.addLine("pathli");
                }
            } else {
                runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_3, 0.5);
                runMotorToPosition(elbowMotor, ELBOW_LEVEL_3, 0.3);
                // 15 for level 3
                moveInches(15, 0.7);
                while(shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                    telemetry.addLine("pathli");
                }
            }

        grabberServo.setPosition(0);
        pivot(95, 0.9);
       // moveInches(72, 1.0);

    }

    public void dropElement() {
        grabberServo.setPosition(GRABBER_OUT);
    }
}
