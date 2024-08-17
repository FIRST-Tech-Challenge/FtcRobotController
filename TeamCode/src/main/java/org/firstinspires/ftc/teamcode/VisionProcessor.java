package org.firstinspires.ftc.teamcode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NewStuff.PropDetector;
import org.firstinspires.ftc.teamcode.NewStuff.PropProcessor;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionProcessor {

    PropProcessor propProcessor;
    AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;

    public OpModeUtilities opModeUtilities;

    public boolean isRedAlliance;

    public VisionProcessor(OpModeUtilities opModeUtilities, boolean isRedAlliance) {
        this.opModeUtilities = opModeUtilities;
        this.isRedAlliance = isRedAlliance;
        setUpVisionPortal();
    }

    private void setUpVisionPortal() {
        propProcessor = new PropProcessor(opModeUtilities.getTelemetry(), isRedAlliance ? PropDetector.ALLIANCE_COLOR.RED : PropDetector.ALLIANCE_COLOR.BLUE);
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setLiveViewContainerId(VisionPortal.DEFAULT_VIEW_CONTAINER_ID)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(false)
                .setAutoStopLiveView(true)
                .setCamera(opModeUtilities.getHardwareMap().get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .addProcessor(propProcessor)
                .build();
    }
}
