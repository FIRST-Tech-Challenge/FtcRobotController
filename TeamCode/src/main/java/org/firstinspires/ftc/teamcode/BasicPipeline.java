package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
// l bozo
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.Arrays;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
public class BasicPipeline extends OpenCvPipeline {
    private final Mat dilateOut = new Mat();
    private final Mat cvtColorOut = new Mat();
    private final Point zeroPoint = new Point(0, 0);

    public static Rect cropRect = new Rect(1, 50, 100, 100); // before camera change it was 590, 225, 50, 50
    private Mat croppedImg = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // ERODE -----------------------------------
        Imgproc.dilate(input, dilateOut, new Mat(), zeroPoint, 10, Core.BORDER_CONSTANT);

        // THRESHOLD -------------------------------
        Imgproc.cvtColor(dilateOut, cvtColorOut, Imgproc.COLOR_RGBA2RGB);

        // CROPPING --------------------------------
        croppedImg = cvtColorOut.submat(cropRect);
        return croppedImg;
    }

    public int getAnalysis(Telemetry telemetry) {
        return 0; // dummy
    }
}
