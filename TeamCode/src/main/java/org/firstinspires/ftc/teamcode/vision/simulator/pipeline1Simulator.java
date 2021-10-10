package org.firstinspires.ftc.teamcode.vision.simulator;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.lang.Math;
import java.util.ArrayList;
import java.util.Collections;
public class pipeline1Simulator extends OpenCvPipeline {
    private Telemetry telemetry;
    //We declare the mats ontop so we can reuse them later to avoid memory leaks
    private Mat matYCrCb = new Mat();
    private Mat matCbBottom = new Mat();
    private Mat matCbTop = new Mat();
    private Mat topBlock = new Mat();
    private Mat bottomBlock = new Mat();

    //Where the average CB value of the rectangles are stored
    private double topAverage;
    private double bottomAverage;

    //The max difference allowed inside the rectangles
    private int threshold = 15;

    //The position related to the screen
    private double topRectWidthPercentage = 0.25;
    private double topRectHeightPercentage = 0.25;
    private double bottomRectWidthPercentage = 0.75;
    private double bottomRectHeightPercentage = 0.75;

    //The width and height of the rectangles in terms of pixels
    private int rectangleWidth = 10;
    private int rectangleHeight = 10;

    public pipeline1Simulator(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        /**
         *input which is in RGB is the frame the camera gives
         *We convert the input frame to the color space matYCrCb
         *Then we store this converted color space in the mat matYCrCb
         *For all the color spaces go to
         *https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
         */
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addData("Telemetry","Telemetry");
        telemetry.addData("topBoxAverage",topAverage);
        telemetry.addData("bottomBoxAverage",bottomAverage);
        // USE Z-SCORE
        telemetry.update();
        //The points needed for the rectangles are calculated here
        Rect topRect = new Rect(
                (int) (matYCrCb.width() * topRectWidthPercentage),
                (int) (matYCrCb.height() * topRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );

        Rect bottomRect = new Rect(
                (int) (matYCrCb.width() * bottomRectWidthPercentage),
                (int) (matYCrCb.height() * bottomRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );

        //The rectangle is drawn into the mat
        drawRectOnToMat(input, topRect, new Scalar(255, 0, 0));
        drawRectOnToMat(input, bottomRect, new Scalar(100, 255, 0));

        //We crop the image so it is only everything inside the rectangles and find the cb value inside of them
        topBlock = matYCrCb.submat(topRect);
        bottomBlock = matYCrCb.submat(bottomRect);
        Core.extractChannel(bottomBlock, matCbBottom, 0);
        Core.extractChannel(topBlock, matCbTop, 0);

        //We take the average
        Scalar bottomMean = Core.mean(matCbBottom);
        Scalar topMean = Core.mean(matCbTop);

        bottomAverage = bottomMean.val[0];
        topAverage = topMean.val[0];

        //return the mat to be shown onto the screen
        return input;
    }

    /**
     * Draw the rectangle onto the desired mat
     *
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     */
    private void mostDifferent(double box1, double box2, double box3) {
        private double boxMean = (box1+box2+box3)/3;
        private double diffBox1 = Math.abs(boxMean - box1);
        private double diffBox2 = Math.abs(boxMean - box2);
        private double diffBox3 = Math.abs(boxMean - box3);
        ArrayList<Double> set = new ArrayList<>();
        set.add(diffBox1);
        set.add(diffBox2);
        set.add(diffBox3);
        private double boxNum = Collections.max(set);
        return set.indexOf(boxNum) + 1;
    }
    private void drawRectOnToMat(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect, color, 1);
    }

    public double getTopAverage() {
        return topAverage;
    }

    public double getBottomAverage() {
        return bottomAverage;
    }

    public void setThreshold(int threshold) {
        this.threshold = threshold;
    }

    public int getThreshold() {
        return threshold;
    }

    public void setTopRectWidthPercentage(double topRectWidthPercentage) {
        this.topRectWidthPercentage = topRectWidthPercentage;
    }

    public void setTopRectHeightPercentage(double topRectHeightPercentage) {
        this.topRectHeightPercentage = topRectHeightPercentage;
    }

    public void setBottomRectWidthPercentage(double bottomRectWidthPercentage) {
        this.bottomRectWidthPercentage = bottomRectWidthPercentage;
    }

    public void setBottomRectHeightPercentage(double bottomRectHeightPercentage) {
        this.bottomRectHeightPercentage = bottomRectHeightPercentage;
    }

    public void setRectangleWidth(int rectangleWidth) {
        this.rectangleWidth = rectangleWidth;
    }

    public void setRectangleHeight(int rectangleHeight) {
        this.rectangleHeight = rectangleHeight;
    }
}
