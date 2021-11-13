package org.firstinspires.ftc.teamcode.Enhancement.Subsystems.Vision.DetectMarker;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Enhancement.Config.VisionConfig;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class DetectMarker {
    HardwareMap hardwareMap;
    OpenCvInternalCamera robotCamera;
    MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;
    QuickTelemetry quickTelemetry;

    public DetectMarker(HardwareMap hardwareMap, OpenCvInternalCamera camera, QuickTelemetry quickTelemetry) {
        this.hardwareMap = hardwareMap;
        this.robotCamera = camera;
        this.quickTelemetry = quickTelemetry.newQuickTelemetryFile("Detect Marker Pipeline");
    }

    /**
     * This method waits until the search for the marker is done, and then it return the marker
     * location. It waits until the marker is found, then it returns the marker location.
     *
     * @return Where the marker is
     * @see DetectMarkerPipeline#getMarkerLocation()
     */
    public MarkerLocation DetectMarkerRun() {
        DetectMarkerPipeline detectMarkerPipeline = new DetectMarkerPipeline(quickTelemetry);
        robotCamera.setPipeline(detectMarkerPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        robotCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(VisionConfig.CAMERA_WIDTH, VisionConfig.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                markerLocation = MarkerLocation.NOT_FOUND;
            }
        });
        if (!(markerLocation == MarkerLocation.NOT_FOUND)) {
            markerLocation = detectMarkerPipeline.getMarkerLocation();
        }
        return markerLocation;
    }
}
