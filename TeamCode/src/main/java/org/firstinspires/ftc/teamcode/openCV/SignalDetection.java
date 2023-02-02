package org.firstinspires.ftc.teamcode.openCV;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.TimeUnit;

public class SignalDetection extends OpenCvPipeline {

    public enum Color {
        ORANGE,
        PURPLE,
        GREEN
    }

    Telemetry.Item telemetry = null;

    ElapsedTime time = null;
    long msUntilDetected = 0;

    // Initialize values
    String cameraName;
    Mat mat = new Mat();

    private Color color;
    private Color c;

    // Threshold for when element is considered visible
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    static Rect RECTANGLE = new Rect(
            new Point(130,80),
            new Point(190,160)
    );

    // Constructor
    public SignalDetection(String cn, Telemetry.Item t, ElapsedTime runtime) {
        telemetry = t;

        //  Update local variables
        cameraName = cn;

        c = Color.GREEN;
        color = Color.GREEN;

        time = runtime;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);

        // Orange YCrCb
        Scalar lowOrange = new Scalar(0, 140, 0);
        Scalar highOrange = new Scalar(255, 255, 100);

        // Purple YCrCb
        Scalar lowPurple = new Scalar(0, 128, 128);
        Scalar highPurple = new Scalar(255, 180, 255);

        // Green YCrCb
        Scalar lowGreen = new Scalar(0, 0, 0);
        Scalar highGreen = new Scalar(255, 110, 140);

        Scalar lowColor = new Scalar(0, 0, 0);
        Scalar highColor = new Scalar(0, 0, 0);

        boolean detected = false;

        switch (c) {
            case ORANGE:
                c = Color.PURPLE;
                break;
            case PURPLE:
                c = Color.GREEN;
                break;
            case GREEN:
                c = Color.ORANGE;
                break;
        }

        switch (c) {
            case ORANGE:
                lowColor = lowOrange;
                highColor = highOrange;
                break;
            case PURPLE:
                lowColor = lowPurple;
                highColor = highPurple;
                break;
            case GREEN:
                lowColor = lowGreen;
                highColor = highGreen;
                break;
        }

        Core.inRange(mat, lowColor, highColor, mat); //Update mat to show black and white areas

        Mat rect = mat.submat(RECTANGLE);

        double colorValue = Core.sumElems(rect).val[0] / RECTANGLE.area() / 255;

        rect.release();

        detected = colorValue > PERCENT_COLOR_THRESHOLD;

        if (detected) {
            color = c;
            msUntilDetected = time.now(TimeUnit.MILLISECONDS);
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorElement = new Scalar(0, 255, 0);
        Scalar colorEmpty = new Scalar(255, 0, 0);

        Scalar colorRect = detected ? colorElement : colorEmpty;

        Imgproc.rectangle(mat, RECTANGLE, colorRect);
        return mat;
    }

    public Color getColor() {
        return color;
    }

    public long getMsUntilDetected() {
        return msUntilDetected;
    }
}












