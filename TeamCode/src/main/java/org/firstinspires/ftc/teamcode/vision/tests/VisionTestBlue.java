package org.firstinspires.ftc.teamcode.vision.tests;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionTestBlue extends OpenCvPipeline {

    // Declare the scalars and mats we will be using
    private Scalar low;
    private Scalar high;

    private Mat mask;
    private Mat frame;
    private Mat output;
    private Point centroid;

    // Initializes color bounds declared above to defaults
    public VisionTestBlue() {
        //red
//        low = new Scalar(31, 152, 95);
//        high = new Scalar(162, 255, 128);

        //blue
        low = new Scalar(0, 0, 0);
        high = new Scalar(190, 105, 160);

    }

    // Allows for user to set their own color bounds
    public VisionTestBlue(Scalar lowerBound, Scalar upperBound) {
        low = lowerBound;
        high = upperBound;
    }

    @Override
    public Mat processFrame(Mat input) {

        frame = new Mat();
        output = new Mat();
        mask = new Mat(output.rows(), output.cols(), CvType.CV_8UC4);

        // Converts the colorspace from RGB to YCrCb
        Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2YCrCb);
        // Creates a mask of all pixels within the specified color bounds
        Core.inRange(output, low, high, mask);
        // Puts the mask over the original image
        Core.bitwise_and(input, input, frame, mask);

        // Blurs the image to smooth it out and reduce unwanted pixels
        Imgproc.GaussianBlur(mask, mask, new Size(11, 15), 10000000.0);

        // Initialize the Mat and ArrayList and finds the contours on the image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draws contours onto the frame from the list created above. During this it searches for
        // the biggest contour and saves it to the variable biggest.
        if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
            MatOfPoint biggest = new MatOfPoint();
            for (int index = 0; index >= 0; index = (int) hierarchy.get(0, index)[0]) {
                Imgproc.drawContours(frame, contours, index, new Scalar(88, 0, 0), 2);
                if (index == 0)
                    biggest = contours.get(index);
                else if (contours.get(index).size().area() > contours.get(index - 1).size().area())
                    biggest = contours.get(index);
            }

            // Creates a point and sets it to the approximate center of the largest contour
            Moments moments = Imgproc.moments(biggest);
            centroid = new Point();

            centroid.x = moments.get_m10() / moments.get_m00();
            centroid.y = moments.get_m01() / moments.get_m00();
            Rect rect = new Rect((int) centroid.x, (int) centroid.y, 10, 10);
            Imgproc.rectangle(frame, rect, new Scalar(0, 255, 255));

        }

        mask.release();

        return frame;
    }

    // Returns the center of the largest contour
    public Point getCentroid() {
        return centroid;
    }

    // Allows the user to set lower and upper bounds after the file has been initialized
    public void setLowerBound(Scalar low) {
        this.low = low;
    }

    public void setUpperBound(Scalar high) {
        this.high = high;
    }

    public void setLowerAndUpperBounds(Scalar low, Scalar high) {
        this.low = low;
        this.high = high;
    }

}