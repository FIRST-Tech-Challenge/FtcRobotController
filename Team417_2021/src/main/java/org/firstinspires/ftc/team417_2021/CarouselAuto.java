package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "storage unit auto")
public class CarouselAuto extends MasterAutonomous {
    int allianceSide = 1;
    int barcodeIndex = 0;
    boolean farSide = false;
    int x;
    int offset = 200;
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
                sleep(300);
            }

            x = barcodeDetector.x;

            offset = 95;

            if (gamepad1.x) {
                offset -= 5;
                sleep(300);
            } else if (gamepad1.b) {
                offset += 5;
                sleep(300);
            }
            if (x < 213 - offset) { // prob need to decrease
                barcodeIndex = 0;
            } else if (x > 213 - offset && x < 486 - offset) {
                barcodeIndex = 1;
            } else if (x > 486 - offset) {
                barcodeIndex = 2;
            }
            //}
            //barcodeIndex = barcodeDetector.index;
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

        // get to the shipping hub

        moveAtAngle(8, 0.7, 0);
        pivot(-90 * allianceSide, 0.5);
        moveAtAngle(21, 0.7, -90 *allianceSide);
        pivot(0, 0.5);
        moveAtAngle(29, 0.7, 0);
        pivot(90 * allianceSide, 0.4);

        if (barcodeIndex == 0) {

            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_1, 0.5);
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_1, 0.5);
            while (shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                telemetry.addLine(" ");
            }
            moveAtAngle(28, 0.7, 90 * allianceSide);

            grabberServo.setPosition(GRABBER_OUT);
            moveAtAngle(-26, 0.7, 90 * allianceSide);
            // go back
            grabberServo.setPosition(GRABBER_IN);

            runMotorToPosition(shoulderMotor, 0, 0.5);
            runMotorToPosition(elbowMotor, 0, 0.5);

            pivot(-180, 0.5);


        } else if (barcodeIndex == 1) {

            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_2, 0.5);
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_2, 0.5);
            while (shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                telemetry.addLine(" ");
            }
            moveAtAngle(28, 0.7, 90 * allianceSide);
            grabberServo.setPosition(GRABBER_OUT);
            moveAtAngle(-24, 0.7, 90 * allianceSide);
            // go back
            grabberServo.setPosition(GRABBER_IN);

            runMotorToPosition(shoulderMotor, 0, 0.5);
            runMotorToPosition(elbowMotor, 0, 0.5);

            pivot(-180, 0.5);

        } else {
            // moveInches(10, 0.7);
            runMotorToPosition(shoulderMotor, SHOULDER_LEVEL_3, 0.5);
            runMotorToPosition(elbowMotor, ELBOW_LEVEL_3, 0.5);
            while (shoulderMotor.isBusy() || elbowMotor.isBusy()) {
                telemetry.addLine(" ");
            }
            moveAtAngle(31, 0.7, 90 * allianceSide);
            grabberServo.setPosition(GRABBER_OUT);
            sleep(1000);
            moveAtAngle(-27, 0.7, 90 * allianceSide);
            // go back
            grabberServo.setPosition(GRABBER_IN);

            runMotorToPosition(shoulderMotor, 0, 0.5);
            runMotorToPosition(elbowMotor, 0, 0.5);

            pivot(-180, 0.5);
        }

        if (allianceSide == 1) {
            blueDeliverDuck();
        }

        else if (allianceSide == -1) {
            redDeliverDuck();
        }
    }

    public void redDeliverDuck () throws InterruptedException{
        moveAtAngle(32, 0.7, -180);
        pivot(110, 0.5);
        motorFL.setPower(0.3);
        motorFR.setPower(0.3);
        motorBL.setPower(0.3);
        motorBR.setPower(0.3);
        sleep(1000);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        carouselMotor.setPower(-1);
        sleep(3000);
        carouselMotor.setPower(0);
        moveAtAngle(-6, 0.5, 110);
        pivot(-180, 0.5);
        moveAtAngle(-11,0.5, -180);
    }
    public void blueDeliverDuck () throws InterruptedException {
        moveAtAngle(41, 0.7, -180);
        carouselMotor.setPower(1);
        sleep(2500);

        while (carouselMotor.isBusy()) {
            telemetry.addLine("a");
        }

        carouselMotor.setPower(0);
        moveAtAngle(-24, 0.7, -180);
    }
}
