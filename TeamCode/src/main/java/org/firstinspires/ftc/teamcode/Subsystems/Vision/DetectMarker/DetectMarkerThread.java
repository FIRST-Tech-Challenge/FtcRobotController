package org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AllianceColor;
import org.firstinspires.ftc.teamcode.Config.MainConfig;
import org.firstinspires.ftc.teamcode.Config.VisionConfig;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class DetectMarkerThread implements Runnable{

    Robot robot;
    HardwareMap hardwareMap;
    OpenCvInternalCamera robotCamera;
    private final AllianceColor allianceColor = MainConfig.getAllianceColor();
    MarkerLocation markerLocation = MarkerLocation.NOT_FOUND;

    public DetectMarkerThread(Robot robot, OpenCvInternalCamera camera) {
        this.robot = robot;
        this.hardwareMap = robot.getOpMode().hardwareMap;
        this.robotCamera = camera;

    }

    /**
     * This method waits until the search for the marker is done, and then it return the marker
     * location. It waits until {@link DetectMarker#getSearchStatus() } returns
     * {@link SearchStatus#FOUND}. The markerLocation is updated every 1 second
     *
     * @see DetectMarker#getMarkerLocation()
     */
    @Override
    public void run() {
        DetectMarker detectMarker = new DetectMarker(robot, allianceColor);
        robotCamera.setPipeline(detectMarker);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        robotCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robotCamera.startStreaming(VisionConfig.CAMERA_WIDTH, VisionConfig.CAMERA_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        while (detectMarker.getSearchStatus() != SearchStatus.FOUND) {
            markerLocation = detectMarker.getMarkerLocation();
        }
    }

    public MarkerLocation getMarkerLocation() {
        return this.markerLocation;
    }
}
