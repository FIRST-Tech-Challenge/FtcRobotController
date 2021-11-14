package org.firstinspires.ftc.team417_2021;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        if (barcodeIndex == 0) {
            // highest level
            shoulderMotor.setTargetPosition(-1300);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setPower(0.5);

            wristServo.setPosition(0.4);

            /*elbowMotor.setTargetPosition(1700);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setPower(0.5);*/
        } else if (barcodeIndex == 1) {
            // middle level
            shoulderMotor.setTargetPosition(-1600);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setPower(0.5);

            /*elbowMotor.setTargetPosition(1700);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setPower(0.5);*/
        } else {
            // lowest level
            shoulderMotor.setTargetPosition(-1500);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setPower(0.5);

            /*elbowMotor.setTargetPosition(1700);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setPower(0.5);*/
        }
        // ----------------- MOVING -----------------
        if (!farSide) {
            move3(2, 0.7);
            pivot(-10 * allianceSide, 0.7);
            move3(4, 0.7);
            pivot(-90 * allianceSide, 0.7);
            move3(22, 0.4);
            pivot(0, 0.7);
            if (barcodeIndex == 0) {
                move3(4, 0.7);
            }
            move3(19, 0.4);
            // drop element here
            dropElement();
            //sleep(5000);
            move3(-10, 0.7);
            pivot(90 * allianceSide, 0.7);
            // back up into parking zone here
            move3(70, 1.0);
        } else {
            move3(6, 0.7);
            pivot(90 * allianceSide, 0.7);
            move3(20, 0.4);
            pivot(0, 0.7);
            move3(2, 0.4);
            // drop element
            dropElement();
            move3(-5, 0.7);
            pivot(90 * allianceSide, 0.7);
            // back up into parking zone

        }


    }

    public void dropElement() {
        /*if (barcodeIndex == 0) {
            // highest level
            shoulderMotor.setTargetPosition(-1300);
            shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulderMotor.setPower(0.5);

            elbowMotor.setTargetPosition(1900);
            elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbowMotor.setPower(0.5);
        } else if (barcodeIndex == 1) {
            // middle level
        } else {
            // lowest level
        }*/
        grabberServo.setPosition(GRABBER_OUT);
    }
}
