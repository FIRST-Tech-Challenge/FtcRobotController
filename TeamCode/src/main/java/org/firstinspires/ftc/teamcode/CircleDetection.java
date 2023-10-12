package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CircleDetection extends OpenCvPipeline {
    enum BallPosition {LEFT, CENTER, RIGHT, UNDEFINED};
    private BallPosition ballPosition = BallPosition.UNDEFINED;
    private boolean detectionRed = true;
    Mat grayMat = new Mat();
    Mat hsvMaskedMat = new Mat();
    Mat mask = new Mat();
    Mat mask1 = new Mat();
    Mat mask2 = new Mat();
    Mat hsvMat = new Mat();
    Mat subMat = new Mat();
    double[] data;
    private int numCirclesFound = 0;
    private Point circleCenter = new Point(0.0, 0.0);

    public CircleDetection(boolean detectionRed) {
        this.detectionRed = detectionRed;
    }

    public int NumCirclesFound()
    {
        return numCirclesFound;
    }
    public Point CircleCenter()
    {
        return circleCenter;
    }

    public boolean CircleFound()
    {
        return ballPosition != BallPosition.UNDEFINED;
    }
    public BallPosition GetBallPosition()
    {
        return ballPosition;
    }

    public void SetBallPosition(BallPosition ballPosition)
    {
        this.ballPosition = ballPosition;
    }
    @Override
    public Mat processFrame(Mat input) {
        subMat = input.submat(new Rect(100, 250, 1000, 300));
        Imgproc.cvtColor(subMat, hsvMat, Imgproc.COLOR_RGB2HSV);
        if (detectionRed) {
            Core.inRange(hsvMat, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1); // RED 1
            Core.inRange(hsvMat, new Scalar(160, 70, 50), new Scalar(180, 255, 255), mask2); // RED 2
            Core.bitwise_or(mask1, mask2, mask);
        } else {
            Core.inRange(hsvMat, new Scalar(92, 60, 0), new Scalar(123, 255, 255), mask); //BLUE
        }
        hsvMaskedMat.release();
        Core.bitwise_and(subMat, subMat, hsvMaskedMat, mask);

        Imgproc.cvtColor(hsvMaskedMat, grayMat, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(grayMat, grayMat, new org.opencv.core.Size(15.0, 15.0), 2, 2);
        Mat circles = new Mat();
        Imgproc.HoughCircles(grayMat, circles, Imgproc.HOUGH_GRADIENT, 1, 300, 120, 25);

        numCirclesFound = circles.cols();

        for (int i = 0; i < numCirclesFound; i++) {
             data = circles.get(0, i);
            circleCenter = new Point(Math.round(data[0])+100, Math.round(data[1])+250);
            ballPosition = circleCenter.x < 427 ? BallPosition.LEFT : (circleCenter.x > 853 ? BallPosition.RIGHT : BallPosition.CENTER);
        }
        return input;
    }
    public double[] getCircleData() {
        double circlesRadius = data[2];
        return data;
    }
}
