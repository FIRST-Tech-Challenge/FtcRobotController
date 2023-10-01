package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class TestAuto extends LinearOpMode {
    OpenCvWebcam webcam;
    private MarkerDetector detector;
    private MarkerDetector.MARKER_POSITION position;
    private double LEFT_CR_AVG;

    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        elapsedTime = new ElapsedTime();

        detector = new MarkerDetector();
        webcam.setPipeline(detector);
        webcam.openCameraDevice();
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        position = detector.position;

        while (!isStarted() || opModeIsActive()) {
            position = detector.position;
            LEFT_CR_AVG = detector.avgLeftCr;
            telemetry.addData("position: ", position);
            telemetry.update();
            telemetry.addLine(String.valueOf(LEFT_CR_AVG));
            telemetry.addLine(String.valueOf(detector.leftCrTotal));

            if (elapsedTime.milliseconds() >= 2000) {
                telemetry.addData("position: ", position);
                telemetry.addLine(String.valueOf(LEFT_CR_AVG));
                telemetry.addLine(String.valueOf(detector.leftCrTotal));
                telemetry.update();
                elapsedTime.reset();
            }
        }

        if (position == MarkerDetector.MARKER_POSITION.LEFT) {

        }

    }


}