package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import android.graphics.Canvas;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class SampleDetectorPipelineGreyscale implements VisionProcessor {
    private double angle;
    public boolean isBlue;
    public Scalar hsvLowerYellow = new Scalar(20,100,100);
    public Scalar hsvUpperYellow = new Scalar(30,255,255);
    public Scalar hsvLowerBlue = new Scalar(110,50,50);
    public Scalar hsvUpperBlue = new Scalar(130,255,255);
    public Scalar hsvLowerRed = new Scalar(90,100,100);
    public Scalar hsvUpperRed = new Scalar(180,255,255);
    public Scalar hsvLowerTeam;
    public Scalar hsvUpperTeam;
    public SampleDetectorPipelineGreyscale(boolean isBlue) {
        this.isBlue = isBlue;
        hsvLowerTeam = isBlue ? hsvLowerBlue : hsvLowerRed;
        hsvUpperTeam = isBlue ? hsvUpperBlue : hsvUpperRed;
    }

    private Mat generateMask(Mat frame) {
        Mat maskYellow = new Mat();
        Mat maskTeam = new Mat();
        Mat returnee = new Mat();
        Core.inRange(frame,hsvLowerYellow,hsvUpperYellow,maskYellow);
        Core.inRange(frame,hsvLowerTeam,hsvUpperTeam,maskTeam);
        Core.bitwise_or(maskYellow,maskTeam,returnee);

        //Memory management for the cool kids
        maskYellow.release();
        maskTeam.release();

        return returnee;
    }

    private Mat generateResult(Mat frame, Mat mask) {
        Mat result = new Mat();
        Core.bitwise_and(frame, frame, result, mask);
        return result;
    }

    private double bestFitLineSlope(double[] x, double[] y) {
        int n = x.length;
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }

        return (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    }

    @Override
    public void init(int use, int less, CameraCalibration useless) {
        // We don't need to initialize anything, so this method is empty
    }

    @Override
    public Object processFrame(Mat frame, long useless) {
        // Separate out the colored pixels and set them aside
        Imgproc.cvtColor(frame,frame,Imgproc.COLOR_RGB2HSV);
        Mat mask = generateMask(frame);
        Mat result = generateResult(frame,mask);

        Mat gray = new Mat();
        Imgproc.cvtColor(result,gray,Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(gray,gray,Imgproc.COLOR_RGB2GRAY); //Convert to grayscale for convenience

        // Find all non-black pixels
        Mat nonZero = new Mat();
        Core.findNonZero(gray,nonZero);

        // Extract the x and y coordinates of the colored pixels
        int totalPoints = nonZero.rows();
        double[] x = new double[totalPoints];
        double[] y = new double[totalPoints];

        // Get the x and y coordinates from the screen
        for (int i = 0; i < totalPoints; i++) {
            double[] point = nonZero.get(i, 0); // Each row contains (x, y)
            x[i] = point[0]; // x-coordinate
            y[i] = point[1]; // y-coordinate
        }

        double slope = bestFitLineSlope(x,y); // Get the slope of the best fit line
        // NGL, ChatGPT wrote the formula for the slope, don't question it

        //Memory management for the cool kids
        mask.release();
        result.release();
        gray.release();
        nonZero.release();

        angle = Math.atan(slope) * 180 / Math.PI;
        // Since slope = dy/dx, and theta = arctan(dy/dx), we can find the angle from the slope
        // We convert the angle from radians to degrees and return it

        return null;
    }

    @Override
    public void onDrawFrame(Canvas b, int r, int u, float h, float w, Object ot) {
        // We don't need to draw anything, so this method is empty
    }

    public double getAngle() {
        return angle;
    }


}