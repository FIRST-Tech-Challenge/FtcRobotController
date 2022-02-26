package org.firstinspires.ftc.teamcode.core.robot.vision.robot;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/*
red
bottom height = 0.2
bottom width = 0.805
middle height = 0.3
middle width = 0.534
top height = 0.45
top width = 0.195

blue
bottom height = 0.41
bottom width = 0.75
middle height = 0.315
middle width = 0.37
top height 0.25
top width = 0.08
 */
@Config
public class TsePipeline extends OpenCvPipeline {
    public TsePipeline(boolean isRed) {
        if (isRed) {
            bottomRectHeightPercentage = 0.15D;
            bottomRectWidthPercentage = 0.79D;
            middleRectHeightPercentage = 0.23D;
            middleRectWidthPercentage = 0.56D;
            topRectHeightPercentage = 0.35D;
            topRectWidthPercentage = 0.2D;
        } else {
            bottomRectHeightPercentage = 0.31D;
            bottomRectWidthPercentage = 0.72D;
            middleRectHeightPercentage = 0.26D;
            middleRectWidthPercentage = 0.35D;
            topRectHeightPercentage = 0.2D;
            topRectWidthPercentage = 0.05D;
        }
    }
    public TsePipeline() {}

    private final Scalar red = new Scalar(255,0,0);
    private final Scalar yellow = new Scalar(255,255,0);
    private final Mat matYCrCb = new Mat();
    private final Mat matCbBottom = new Mat();
    private final Mat matCbMiddle = new Mat();
    private final Mat matCbTop = new Mat();
    private final Mat matCbBottom1 = new Mat();
    private final Mat matCbMiddle1 = new Mat();
    private final Mat matCbTop1 = new Mat();
    private final Mat matCbBottom2 = new Mat();
    private final Mat matCbMiddle2 = new Mat();
    private final Mat matCbTop2 = new Mat();

    //Where the average CB value of the rectangles are stored
    private double topAverage = 0;
    private double middleAverage = 0;
    private double bottomAverage = 0;

    //The position related to the screen
    public static double topRectWidthPercentage = 0.25;
    public static double topRectHeightPercentage = 0.50;
    public static double middleRectWidthPercentage = 0.50;
    public static double middleRectHeightPercentage = 0.50;
    public static double bottomRectWidthPercentage = 0.75;
    public static double bottomRectHeightPercentage = 0.50;
    private int different = 0;
    private int lastFrameValue = 0;
    private boolean isComplete = false;
    private int checks = 0;
    private Pair<Integer, Integer> greatestConfidence = new Pair<>(0, 0);
    private int frameCount = 0;
    private boolean running = false;
    public void startPipeline() {
        different = 0;
        lastFrameValue = 0;
        isComplete = false;
        checks = 0;
        frameCount = 0;
        running = true;
    }
    public void stopPipeline() {
        running = false;
    }
    /**
     * @param input input frame matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
        //The points needed for the rectangles are calculated here
        int rectangleHeight = 10;
        //The width and height of the rectangles in terms of pixels
        int rectangleWidth = 10;
        Rect topRect = new Rect(
                (int) (matYCrCb.width() * topRectWidthPercentage),
                (int) (matYCrCb.height() * topRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );
        Rect middleRect = new Rect(
                (int) (matYCrCb.width() * middleRectWidthPercentage),
                (int) (matYCrCb.height() * middleRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );
        Rect bottomRect = new Rect(
                (int) (matYCrCb.width() * bottomRectWidthPercentage),
                (int) (matYCrCb.height() * bottomRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );
        if (running) {
            Mat topBlock = matYCrCb.submat(topRect);
            Mat middleBlock = matYCrCb.submat(middleRect);
            Mat bottomBlock = matYCrCb.submat(bottomRect);
            Core.extractChannel(topBlock, matCbTop, 0);
            Core.extractChannel(middleBlock, matCbMiddle, 0);
            Core.extractChannel(bottomBlock, matCbBottom, 0);
            Core.extractChannel(topBlock, matCbTop1, 1);
            Core.extractChannel(middleBlock, matCbMiddle1, 1);
            Core.extractChannel(bottomBlock, matCbBottom1, 1);
            Core.extractChannel(topBlock, matCbTop2, 2);
            Core.extractChannel(middleBlock, matCbMiddle2, 2);
            Core.extractChannel(bottomBlock, matCbBottom2, 2);

            Scalar topMeanY = Core.mean(matCbTop);
            Scalar topMeanCr = Core.mean(matCbTop1);
            Scalar topMeanCb = Core.mean(matCbTop2);
            Scalar middleMeanY = Core.mean(matCbMiddle);
            Scalar middleMeanCr = Core.mean(matCbMiddle1);
            Scalar middleMeanCb = Core.mean(matCbMiddle2);
            Scalar bottomMeanY = Core.mean(matCbBottom);
            Scalar bottomMeanCr = Core.mean(matCbBottom1);
            Scalar bottomMeanCb = Core.mean(matCbBottom2);

            topAverage = Math.abs(topMeanY.val[0] - 152.85) + Math.abs(topMeanCr.val[0] - 147.88) + Math.abs(topMeanCb.val[0] - 57);
            middleAverage = Math.abs(middleMeanY.val[0] - 152.85) + Math.abs(middleMeanCr.val[0] - 147.88) + Math.abs(middleMeanCb.val[0] - 57);
            bottomAverage = Math.abs(bottomMeanY.val[0] - 152.85) + Math.abs(bottomMeanCr.val[0] - 147.88) + Math.abs(bottomMeanCb.val[0] - 57);
            different = mostSmall(topAverage, middleAverage, bottomAverage);
            switch (different) {
                case 1:
                    drawRectOnToMat(input, topRect, yellow);
                    drawRectOnToMat(input, middleRect, red);
                    drawRectOnToMat(input, bottomRect, red);
                    break;
                case 2:
                    drawRectOnToMat(input, topRect, red);
                    drawRectOnToMat(input, middleRect, yellow);
                    drawRectOnToMat(input, bottomRect, red);
                    break;
                case 3:
                    drawRectOnToMat(input, topRect, red);
                    drawRectOnToMat(input, middleRect, red);
                    drawRectOnToMat(input, bottomRect, yellow);
                    break;
            }
            if (!isComplete) {
                frameCount++;
                if (different == lastFrameValue || checks == 0) {
                    checks++;
                } else {
                    if (greatestConfidence.second < checks) {
                        greatestConfidence = new Pair<>(different, checks);
                    }
                    checks = 0;
                }

                if (checks >= 5) {
                    isComplete = true;
                }

                if (frameCount >= 20) {
                    different = greatestConfidence.first;
                    isComplete = true;
                }

                lastFrameValue = different;
            }
        } else {
            drawRectOnToMat(input, topRect, red);
            drawRectOnToMat(input, middleRect, red);
            drawRectOnToMat(input, bottomRect, red);
        }
        return input;
    }

    public Pair<Boolean, Integer> differentSpot() {
        return new Pair<>(isComplete, different);
    }

    /**
     * Gets the most different value from 3 doubles.
     *
     * @param val1 double
     * @param val2 double
     * @param val3 double
     * @return Most different, 1-3
     */
    public static int mostDifferent(double val1, double val2, double val3) {
        double valMean = (val1 + val2 + val3) / 3;
        double[] array = {Math.abs(valMean - val1),Math.abs(valMean - val2), Math.abs(valMean - val3)};
        int max = array[0] > array[1] ? 1 : 2;
        return array[2] > array[max-1] ? 3 : max;
    }

    public static int mostSmall(double val1, double val2, double val3) {
        final List<Double> numList = Arrays.asList(val1, val2, val3);
        return numList.indexOf(Collections.min(numList)) + 1;
    }

    /**
     * Draw the rectangle onto the desired mat
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     */
    private void drawRectOnToMat(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect, color, 1);
    }

    /**
     * percentages of all rectangles. it goes top width, top height, middle width, etc.
     */
    @SuppressWarnings("unused")
    public void setRectangles(double topRectWidthPercentage, double topRectHeightPercentage,
                       double middleRectWidthPercentage, double middleRectHeightPercentage,
                       double bottomRectWidthPercentage, double bottomRectHeightPercentage) {
        TsePipeline.topRectWidthPercentage = topRectWidthPercentage;
        TsePipeline.topRectHeightPercentage = topRectHeightPercentage;
        TsePipeline.middleRectWidthPercentage = middleRectWidthPercentage;
        TsePipeline.middleRectHeightPercentage = middleRectHeightPercentage;
        TsePipeline.bottomRectWidthPercentage = bottomRectWidthPercentage;
        TsePipeline.bottomRectHeightPercentage = bottomRectHeightPercentage;
    }
}
