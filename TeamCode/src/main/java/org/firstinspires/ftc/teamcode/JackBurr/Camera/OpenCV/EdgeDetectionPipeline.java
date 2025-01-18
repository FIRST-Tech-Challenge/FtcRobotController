package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class EdgeDetectionPipeline extends OpenCvPipeline {
    // Mat objects for processing
    public Telemetry telemetry;
    private Mat blurred = new Mat();
    public Mat grayMat = new Mat();
    public Mat edges = new Mat();
    public int MIN_AREA = 400;
    public int MAX_AREA = 10000;
    public void init(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    // Angle of the detected sample
    private double detectedAngle = 0;
    public double area = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input frame to grayscale
        Imgproc.cvtColor(input, grayMat, Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(grayMat, blurred, new Size(5, 5), 0);
        Imgproc.Canny(blurred, edges, 50, 150);

// Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            // Filter by size and aspect ratio
            if (area > MIN_AREA && area < MAX_AREA) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                area = rect.size.height * rect.size.width;
                detectedAngle = rect.angle;
                Point[] vertices = new Point[4];
                rect.points(vertices); // Get the rectangle vertices
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2); // Green lines
                }
                // Assuming the sample has a specific size/shape
                telemetry.addLine("Detected Sample:");
                telemetry.addLine("\t Width: " + rect.size.width);
                telemetry.addLine("\t Height: " + rect.size.height);
                telemetry.addLine("\t Angle: " + detectedAngle);
            }
        }
        // Return the image with contours and angle annotations
        return input;
    }

    public double getDetectedAngle() {
        return detectedAngle;
    }
}
