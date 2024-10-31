package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class ColorDetect implements VisionProcessor {

    // Set the YCrCb color ranges for blue based on the slider data
    private final Scalar lowerBlue = new Scalar(55.3, 90.7, 141.7);  // Adjusted blue lower bound
    private final Scalar upperBlue = new Scalar(181.3, 182.8, 255.0); // Adjusted blue upper bound

    // Set the YCrCb color ranges for red based on the slider data
    private final Scalar lowerRed = new Scalar(19.8, 164.0, 68.0);   // Adjusted red lower bound
    private final Scalar upperRed = new Scalar(171.4, 200.0, 120.0); // Adjusted red upper bound

    // Set the YCrCb color ranges for yellow based on the slider data
    private final Scalar lowerYellow = new Scalar(79.3, 140.0, 20.0);  // Adjusted yellow lower bound
    private final Scalar upperYellow = new Scalar(204.0, 225.0, 75.0); // Adjusted yellow upper bound

    // List to store detected colors
    private List<DetectedColor> detectedColors = new ArrayList<>();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Clear the list of detected colors for each frame
        detectedColors.clear();

        // Convert the input image from RGB to YCrCb color space for color detection
        Mat ycrcb = new Mat();
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);

        // Convert the input image to HSV color space for saturation detection
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Step 1: Detect Blue and mask it out
        Mat maskBlue = new Mat();
        Core.inRange(ycrcb, lowerBlue, upperBlue, maskBlue);
        processColorWithSaturation(maskBlue, hsvMat, input, new Scalar(255, 0, 0), "Blue", 500);
        Core.bitwise_not(maskBlue, maskBlue);
        Core.bitwise_and(ycrcb, ycrcb, ycrcb, maskBlue);  // Mask out blue region

        // Step 2: Detect Red on the remaining image
        Mat maskRed = new Mat();
        Core.inRange(ycrcb, lowerRed, upperRed, maskRed);
        processColorWithSaturation(maskRed, hsvMat, input, new Scalar(0, 255, 0), "Red", 500);
        Core.bitwise_not(maskRed, maskRed);
        Core.bitwise_and(ycrcb, ycrcb, ycrcb, maskRed);  // Mask out red region

        // Step 3: Detect Yellow on the remaining image
        Mat maskYellow = new Mat();
        Core.inRange(ycrcb, lowerYellow, upperYellow, maskYellow);
        processColorWithSaturation(maskYellow, hsvMat, input, new Scalar(0, 255, 255), "Yellow", 500);
        Core.bitwise_not(maskYellow, maskYellow);
        Core.bitwise_and(ycrcb, ycrcb, ycrcb, maskYellow);  // Mask out yellow region

        // Release Mat objects to prevent memory leaks
        ycrcb.release();
        hsvMat.release();
        maskBlue.release();
        maskRed.release();
        maskYellow.release();

        return null;
    }

    private void processColorWithSaturation(Mat mask, Mat hsvMat, Mat frame, Scalar boxColor, String colorName, int minArea) {
        // Find contours on the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask.clone(), contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            // Get bounding box for the contour
            Rect rect = Imgproc.boundingRect(contour);

            // Filter out small contours by area
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea > minArea) {
                // Get the average saturation of this region
                double avgSaturation = getAvgSaturation(hsvMat, rect);

                // Draw the bounding box around the region
                Imgproc.rectangle(frame, rect, boxColor, 2);

                // Put text label near the bounding box with the color name and saturation value
                String label = colorName + " - Sat: " + String.format("%.2f", avgSaturation);
                Imgproc.putText(frame, label, new Point(rect.x, rect.y - 5), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, boxColor, 2);

                // Add detected color to the list
                detectedColors.add(new DetectedColor(colorName, rect, contourArea));
            }
        }

        // Release resources
        hierarchy.release();
        for (MatOfPoint contour : contours) {
            contour.release();
        }
    }

    // Function to get the average saturation in a given region
    protected double getAvgSaturation(Mat hsvInput, Rect rect) {
        Mat submat = hsvInput.submat(rect);
        Scalar meanVal = Core.mean(submat);
        return meanVal.val[1]; // Return the saturation channel value
    }

    // Public method to return a list of detected colors, sorted by proximity (area of bounding box)
    public List<DetectedColor> getDetectedColors() {
        // Sort detected colors by area, from largest (closest) to smallest (farthest)
        Collections.sort(detectedColors, new Comparator<DetectedColor>() {
            @Override
            public int compare(DetectedColor c1, DetectedColor c2) {
                return Double.compare(c2.area, c1.area); // Sort descending by area
            }
        });
        return detectedColors;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx , float scaleCanvasDensity, Object userContext) {
        // Draw rectangles on canvas (optional for display purposes)
        // Example drawing code commented out
        // Paint paint = new Paint();
        // paint.setColor(Color.GREEN);
        // paint.setStyle(Paint.Style.STROKE);
        // paint.setStrokeWidth(5);
        // canvas.drawRect(makeGraphicsRect(someRect, scaleBmpPxToCanvasPx), paint);
    }
}
