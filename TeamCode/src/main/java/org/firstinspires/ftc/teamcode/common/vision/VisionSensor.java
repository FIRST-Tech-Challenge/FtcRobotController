package org.firstinspires.ftc.teamcode.common.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.enums.PropDirection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionSensor {
    private LinearOpMode opMode;
    private PropPipeline propProcessor;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public VisionSensor(LinearOpMode opMode) {
        this.opMode = opMode;
        propProcessor = new PropPipeline(this.opMode);
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTagProcessor.setDecimation(2);

        visionPortal = createVisionPortal(propProcessor, aprilTagProcessor);

        visionPortal.resumeLiveView();
    }

    public void goToPropDetectionMode() {
        pauseAprilTagDetection();
        resumePropDetection();
        resumeLiveView();
    }

    public void goToAprilTagDetectionMode() {
        pausePropDetection();
        resumeAprilTagDetection();
        setAprilTagCameraValues();
        pauseLiveView();
    }

    public void goToNoSensingMode() {
        pausePropDetection();
        pauseAprilTagDetection();
        pauseLiveView();
    }

    public PropDirection getPropDirection()
    {
        return (propProcessor.getPropDirection());
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        return(aprilTagProcessor.getDetections());
    }

    protected VisionPortal createVisionPortal(PropPipeline propProcessor, AprilTagProcessor aprilTagProcessor) {
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(propProcessor)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setCameraResolution(new Size(640, 480))
                .build();

        // Make sure camera is streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting");
            opMode.telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
        }
        return (visionPortal);
    }

    private void pausePropDetection() {
        visionPortal.setProcessorEnabled(propProcessor, false);
    }

    private void resumePropDetection() {
        visionPortal.setProcessorEnabled(propProcessor, true);
    }

    private void pauseAprilTagDetection() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, false);

    }

    private void resumeAprilTagDetection() {
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);

    }

    private void pauseLiveView() {
        visionPortal.stopLiveView();
    }

    private void resumeLiveView() {
        visionPortal.resumeLiveView();
    }

    private void setAprilTagCameraValues() {
        // Set camera controls unless we are stopping.
        if (!opMode.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                opMode.sleep(50);
            }
            exposureControl.setExposure((long) 6, TimeUnit.MILLISECONDS);
            opMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            opMode.sleep(20);
        }
    }
}
