package org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.pipelines;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class OpenCVPipeline extends OpenCvPipeline
{
    private Mat blurInput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdInput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private List<MatOfPoint> findContoursOutput = new ArrayList<>();
    private Mat findContoursInput = new Mat();
    private Mat findContoursOutputMat = new Mat();
    private Mat finalContourOutputMat = new Mat();
    private volatile Mat dashboardImage = new Mat();

    private int largestX, largestY;
    private double largestArea;
    private volatile Position lastPosition;
    
    // Constants
    public static int VIEW_OPEN_CV_PIPELINE_STAGE = 4;
    public static double BLUR_RADIUS = 7;
    public static double HUE_MIN = 0;
    public static double HUE_MAX = 90;
    public static double SATURATION_MIN = 120;
    public static double SATURATION_MAX = 255;
    public static double VALUE_MIN = 100;
    public static double VALUE_MAX = 255;
    public static double MIN_CONTOUR_AREA = 2500;
    public static String BLUR = "Box Blur";

    public static int LEFT_THRESHOLD = 107;
    public static int RIGHT_THRESHOLD = 213;

    public OpenCVPipeline() {
        largestX = -1;
        largestY = -1;
        largestArea = -1;
        lastPosition = Position.HOLD;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Step Blur0 (stage 1):
        blurInput = input;
        BlurType blurType = BlurType.get(BLUR);
        double blurRadius = BLUR_RADIUS;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0  (stage 2):
        hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {HUE_MIN, HUE_MAX};
        double[] hsvThresholdSaturation = {SATURATION_MIN, SATURATION_MAX};
        double[] hsvThresholdValue = {VALUE_MIN, VALUE_MAX};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Find_Contours0 (stage 3):
        findContoursInput = hsvThresholdOutput;
        findContours(findContoursInput, findContoursOutput);
        findContoursOutputMat = input.clone();
        for(int i = 0; i < findContoursOutput.size(); i++) {
            Imgproc.drawContours(findContoursOutputMat, findContoursOutput, i, new Scalar(255, 255, 255), 2);
        }

        // Finding largest contour (stage 4):
        finalContourOutputMat = input.clone();
        largestArea = -1;
        largestX = -1;
        largestY = -1;
        int largestContourIndex = -1;
        for(int i = 0; i < findContoursOutput.size(); i++) {
            MatOfPoint contour = findContoursOutput.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if(contourArea > MIN_CONTOUR_AREA && contourArea > largestArea) {
                Moments p = Imgproc.moments(contour, false);
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());

                largestContourIndex = i;
                largestX = x;
                largestY = y;
                largestArea = contourArea;
            }
        }
        if(largestContourIndex != -1) {
            Imgproc.drawContours(finalContourOutputMat, findContoursOutput, largestContourIndex, new Scalar(255, 255, 255), 2);
            Imgproc.drawMarker(finalContourOutputMat, new Point(largestX, largestY), new Scalar(255, 255, 255));
        }

        if(largestX > 0 && largestX < LEFT_THRESHOLD) {
            lastPosition = Position.LEFT;
        } else if(largestX > LEFT_THRESHOLD && largestX < RIGHT_THRESHOLD) {
            lastPosition = Position.MIDDLE;
        } else if(largestX > RIGHT_THRESHOLD && largestX < input.width()) {
            lastPosition = Position.RIGHT;
        } else
            lastPosition = Position.NONE_FOUND;

        switch(VIEW_OPEN_CV_PIPELINE_STAGE) {
            case 0:
                dashboardImage = blurInput;
                break;
            case 1:
                dashboardImage = blurOutput;
                break;
            case 2:
                dashboardImage = hsvThresholdOutput;
                break;
            case 3:
                dashboardImage = findContoursOutputMat;
                break;
            case 4:
                dashboardImage = finalContourOutputMat;
                break;
            default:
                dashboardImage = input;
                break;
        }

        return input;
    }

    public int[] getPosition() {
        return new int[] {largestX, largestY};
    }

    public Mat getDashboardImage() {
        return dashboardImage;
    }

    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    private void findContours(Mat input, List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode = Imgproc.RETR_LIST;
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    public Position getLastPosition() {
        return lastPosition;
    }
}
