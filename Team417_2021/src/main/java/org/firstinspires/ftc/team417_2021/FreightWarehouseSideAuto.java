package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Freight Warehouse Side")
public class FreightWarehouseSideAuto extends MasterAutonomous {
    int allianceSide = 1;
    int barcodeIndex = 0;
    boolean farSide = false;
    int x;
    int offset = 40;
    @Override
    public void runOpMode() throws InterruptedException {
        // set up detection
        OpenCvCamera webcam;
        BarcodeDetectionOpenCV barcodeDetector = new BarcodeDetectionOpenCV();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Side Webcam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
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
                x = barcodeDetector.x;

                if (gamepad1.x) {
                    offset -= 5;
                    sleep(300);
                } else if (gamepad1.b) {
                    offset += 5;
                    sleep(300);
                }
                if (x < 213 - offset) { // prob need to decrease
                    barcodeIndex = 0;
                } else if (x > 213 - offset && x < 400 - offset) {
                    barcodeIndex = 1;
                } else if (x > 400 - offset) {
                    barcodeIndex = 2;
            }

            telemetry.addLine("1 for blue, -1 for red");
            telemetry.addData("Alliance", allianceSide);
            telemetry.addLine("False for starting on side nearest warehouse");
            telemetry.addData("Starting position", farSide);
            telemetry.addData("index", barcodeIndex);
            telemetry.addData("offset", offset);
            telemetry.update();
            idle();
        }

        waitForStart();
        robot.setInitialAngle();

        moveInches(1, 0.5);
        pivot(-22 * allianceSide, 0.5);
        moveInches(10, 0.5);
        pivot(-90 * allianceSide, 0.5);
        sleep(100);
        pivot(-90 * allianceSide, 0.5);

        if (barcodeIndex == 0) {
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_1, 0.5);
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_1, 0.3);
            // 15 for level 3

            while (shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                telemetry.addLine("pathli");
            }

            moveInches(18, 0.5);
            pivot(0, 0.5);
            moveInches(8, 0.5);
            grabberServo.setPosition(GRABBER_OUT);
            sleep(500);
            moveInches(-5, 0.5);
            pivot(90 * allianceSide, 0.5);
            runMotorToPosition(shoulderMotor, 0, 0.5);
            runMotorToPosition(elbowMotor, 0, 0.3);
            moveInches(80, 1);

        } else if (barcodeIndex == 1) {
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_2, 0.5);
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_2, 0.3);
            // 15 for level 3
            while (shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                telemetry.addLine("pathli");
            }
            moveInches(21, 0.5);
            pivot(0, 0.5);

            moveInches(8, 0.5);

            grabberServo.setPosition(GRABBER_OUT);
            sleep(500);

            moveInches(-5, 0.5);
            pivot(90 * allianceSide, 0.5);
            runMotorToPosition(shoulderMotor, 0, 0.5);
            runMotorToPosition(elbowMotor, 0, 0.3);
            moveInches(80, 1);
        } else {
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_3, 0.5);
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_3, 0.3);
            // 15 for level 3
            while (shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                telemetry.addLine("pathli");
            }

            moveInches(21, 0.5);
            pivot(0, 0.5);

            moveInches(10, 0.5);

            grabberServo.setPosition(GRABBER_OUT);
            sleep(500);
            moveInches(-5, 0.5);
            pivot(90 * allianceSide, 0.5);
            runMotorToPosition(shoulderMotor, 0, 0.5);
            runMotorToPosition(elbowMotor, 0, 0.3);
            moveInches(80, 1);
        }
    }

    public void dropElement() {
        grabberServo.setPosition(GRABBER_OUT);
    }
}
