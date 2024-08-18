package org.firstinspires.ftc.teamcode.NewStuff;

import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NewStuff.PropDetector;
import org.firstinspires.ftc.teamcode.NewStuff.PropProcessor;
import org.firstinspires.ftc.teamcode.NewStuff.OpModeUtilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionPortalProcessor {

    private PropProcessor propProcessor;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    OpModeUtilities opModeUtilities;

    boolean isRedAlliance;

    public VisionPortalProcessor(OpModeUtilities opModeUtilities, boolean isRedAlliance) {
        this.opModeUtilities = opModeUtilities;
        this.isRedAlliance = isRedAlliance;
        setUpVisionPortal();
    }

    private void setUpVisionPortal() {
        Log.d("vision", "portal: setting up processors");
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
        visionPortal.setProcessorEnabled(propProcessor, false);
        visionPortal.setProcessorEnabled(aprilTagProcessor,false);
        Log.d("vision", "portal: finished setting up processors");
    }

    public boolean getIsRedAlliance() {
        return isRedAlliance;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public OpModeUtilities getOpModeUtilities() {
        return opModeUtilities;
    }

    public PropProcessor getPropProcessor() {
        return propProcessor;
    }
}
