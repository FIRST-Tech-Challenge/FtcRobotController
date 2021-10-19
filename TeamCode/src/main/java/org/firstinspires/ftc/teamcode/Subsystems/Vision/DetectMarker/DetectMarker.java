package org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.VisionConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class DetectMarker {
    Robot robot;
    HardwareMap hardwareMap;
    OpenCvInternalCamera robotCamera;
    MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;

    public DetectMarker(Robot robot, OpenCvInternalCamera camera) {
        this.robot = robot;
        this.hardwareMap = robot.getOpMode().hardwareMap;
        this.robotCamera = camera;
    }

    /**
     * This method waits until the search for the marker is done, and then it return the marker
     * location. It waits until {@link DetectMarkerPipeline#getSearchStatus() } returns the marker
     * location.
     *
     * @return Where the marker is
     *
     * @see DetectMarkerPipeline#getMarkerLocation()
     */
    public MarkerLocation DetectMarkerRun() {
        DetectMarkerPipeline detectMarkerPipeline = new DetectMarkerPipeline(robot);
        robotCamera.setPipeline(detectMarkerPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        robotCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robotCamera.startStreaming(VisionConfig.CAMERA_WIDTH, VisionConfig.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        markerLocation = detectMarkerPipeline.getMarkerLocation();
        return markerLocation;
    }
}
