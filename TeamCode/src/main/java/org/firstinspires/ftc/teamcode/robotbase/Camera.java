package org.firstinspires.ftc.teamcode.robotbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.function.BooleanSupplier;

public class Camera {
    private final OpenCvWebcam webcam;
    protected RectangleTracking pipeline;

    public Camera(HardwareMap hardwareMap, FtcDashboard dashboard, Telemetry telemetry,
                  BooleanSupplier colorSet) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
                "webcam"), cameraMonitorViewId);
        dashboard.startCameraStream(webcam, 30);

        pipeline = new RectangleTracking(telemetry, colorSet);
        webcam.setPipeline(pipeline);

        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public RectangleTracking getPipeline(){
        return pipeline;
    }

    public double getObject_x(){
        return pipeline.getElementsAnalogCoordinates()[0];
    }

    public double getObject_y(){
        return pipeline.getElementsAnalogCoordinates()[1];
    }
}
