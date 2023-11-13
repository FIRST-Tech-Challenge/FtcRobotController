package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public abstract class PipelineBase extends OpenCvPipeline {
    public OpenCvWebcam webcam;
    public Mat frameTemp;
    public List<MatOfPoint> contoursList = new ArrayList<>();
    public static final Scalar RED = new Scalar(255, 0, 0);
    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar YELLOW = new Scalar(255, 255, 0);
    public static final Scalar BLACK = new Scalar(0, 0, 0);
    public static final Rect mask = new Rect((640 - 80) / 2, (480 - 100) / 2, 50, 80);

    public static boolean isInside(MatOfPoint cont) {
        Point[] contourPoints = cont.toArray();
        for (Point p : contourPoints) {
            if (!mask.contains(p)) {
                return false;
            }
        }
        return true;
    }

    public final void setupWebcam(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);

        webcam.setPipeline(this);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public final void disposeWebcam() {
        try {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        } catch(Exception e) {
            // do nothing
        }
    }
}
