package org.firstinspires.ftc.teamcode.robots.csbot.vision.pipeline;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.robots.csbot.vision.Position;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.TimestampedOpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Iron Reign Coding Team
 */

@Config(value = "AA_PP_6CanVisionPipeline")
public class DPRGCanDetectorPipeline extends TimestampedOpenCvPipeline {
    private Mat cropOutput = new Mat();
    private Mat normalizeInput = new Mat();
    private Mat normalizeOutput = new Mat();
    private Mat blurInput = new Mat();
    private Mat blurOutput = new Mat();
    private Mat hsvThresholdInput = new Mat();
    private Mat hsvThresholdOutput = new Mat();
    private List<MatOfPoint> findContoursOutput = new ArrayList<>();
    private Mat findContoursInput = new Mat();
    private Mat findContoursOutputMat = new Mat();
    private Mat finalContourOutputMat = new Mat();
    private Mat dashboardMat = new Mat();
    private Mat hierarchy = new Mat();
    private volatile Bitmap dashboardBitmap;

    private RotatedRect rotatedRect;
    private volatile int largestX, largestY;
    private double largestArea;
    private volatile Position lastPosition;

    // Constants
    public static int VIEW_OPEN_CV_PIPELINE_STAGE = 6;
    public static int TOP_LEFT_X = 0, TOP_LEFT_Y = 0;
    public static int BOTTOM_RIGHT_X = 320, BOTTOM_RIGHT_Y = 180;
    public static double NORMALIZE_ALPHA = 51.0, NORMALIZE_BETA = 261.0;
    public static double BLUR_RADIUS = 7;
    public static double HUE_MIN = 105, HUE_MAX = 120;
    public static double SATURATION_MIN = 80, SATURATION_MAX = 255;
    public static double VALUE_MIN = 120, VALUE_MAX = 255;
    public static double MIN_CONTOUR_AREA = 50;
    public static String BLUR = "Box Blur";

    public static int LEFT_THRESHOLD = 142;
    public static int RIGHT_THRESHOLD = 211;
    public static int CENTER_LINE = 160;

    public DPRGCanDetectorPipeline() {
        largestX = -1;
        largestY = -1;
        largestArea = -1;
        lastPosition = Position.HOLD;
    }

    volatile List<Target> detectedCans = new ArrayList<>(); //persist the most recent targets for access outside this thread
    List<Target> frameCans = new ArrayList<>(); //building the targets inside processFrame

    @Override
    public Mat processFrame(Mat input, long timestamp) {

        //initialize
        frameCans.clear();

        // Step crop (stage 1):
        cropOutput = input.submat(new Rect(new Point(TOP_LEFT_X, TOP_LEFT_Y), new Point(BOTTOM_RIGHT_X, BOTTOM_RIGHT_Y)));

        // Step Normalize0 (stage 2):
        normalizeInput = cropOutput;
        int normalizeType = Core.NORM_MINMAX;
        double normalizeAlpha = NORMALIZE_ALPHA;
        double normalizeBeta = NORMALIZE_BETA;
        normalize(normalizeInput, normalizeType, normalizeAlpha, normalizeBeta, normalizeOutput);

        // Step Blur0 (stage 3):
        blurInput = normalizeOutput;
        BlurType blurType = BlurType.get(BLUR);
        double blurRadius = BLUR_RADIUS;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0  (stage 4):
        hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {HUE_MIN, HUE_MAX};
        double[] hsvThresholdSaturation = {SATURATION_MIN, SATURATION_MAX};
        double[] hsvThresholdValue = {VALUE_MIN, VALUE_MAX};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Find_Contours0 (stage 5):
        findContoursInput = hsvThresholdOutput;
        findContours(findContoursInput, findContoursOutput);
        findContoursOutputMat = cropOutput;
        for (int i = 0; i < findContoursOutput.size(); i++) {
            Imgproc.drawContours(findContoursOutputMat, findContoursOutput, i, new Scalar(255, 255, 255), 2);
        }

        // process contours (stage 6):
        finalContourOutputMat = cropOutput;
        largestArea = -1;
        largestX = -1;
        largestY = -1;
        int largestContourIndex = -1;
        RotatedRect[] minEllipse = new RotatedRect[findContoursOutput.size()];
        Rect[] boundingBox = new Rect[findContoursOutput.size()];

        for (int i = 0; i < findContoursOutput.size(); i++) {

            MatOfPoint contour = findContoursOutput.get(i);
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea > MIN_CONTOUR_AREA) {
                Moments p = Imgproc.moments(contour, false);

                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());

                if (contourArea > largestArea) {
                    largestContourIndex = i;
                    largestX = x;
                    largestY = y;
                    largestArea = contourArea;
                }

                //by fitting a minimal ellipse to a contour, we get its angle, length and width
                if (findContoursOutput.get(i).rows() > 5) { //this test was from an example - prolly means fitEllipse will fail on a too-small contour
                    minEllipse[i] = Imgproc.fitEllipse(new MatOfPoint2f(findContoursOutput.get(i).toArray()));
                } else minEllipse[i] = new RotatedRect();

                boundingBox[i] = Imgproc.boundingRect(new MatOfPoint2f(findContoursOutput.get(i).toArray()));

                Target newTarget = new Target(timestamp, i, new Vector2d(x, y), 0);
                newTarget.setAreaPixels(contourArea);
                newTarget.setFittedRect(minEllipse[i]);
                newTarget.setOrientation(minEllipse[i].angle);
                newTarget.setHeightPixels(minEllipse[i].size.height);
                newTarget.setWidthPixels(minEllipse[i].size.width);
                newTarget.setAspectRatio(newTarget.getWidthPixels() / newTarget.getHeightPixels());
                //an upright can will almost always have an aspect ratio hovering around .6
                //a can on its side will be a little higher even when lying orthogonal to the camera
                //even though the width is still the smaller value because it is perpendicular to the major axis
                //todo a better test of uprightness would be to look at the orientation of the can
                //an upright can seems to have an orientation that is +/- 15 degrees from 0 or 180 (flipped orientation)
                //a can on its side will have an orientation that is closer to 90 degrees or probably 270 (not witnessed yet)
                if (newTarget.getAspectRatio()<.75) newTarget.setUpright(true);
                else newTarget.setUpright(false);
                if (newTarget.isUpright()) //if it is upright, the height of the contour's bounding box is more stable than the fitted ellipse - so switch over to that
                    newTarget.setHeightPixels(boundingBox[i].height);
                //add to the list of targets
                frameCans.add(newTarget);
            }
        }
        //we are done building the local list of targets - copy to the list that the robot can access
        detectedCans = new ArrayList<>(frameCans);

        if (largestContourIndex != -1) {
            Imgproc.drawContours(finalContourOutputMat, findContoursOutput, largestContourIndex, new Scalar(255, 255, 255), 2);
            Imgproc.drawMarker(finalContourOutputMat, new Point(largestX, largestY), new Scalar(0, 255, 0));
            for (Target can : frameCans) {
                Imgproc.ellipse(finalContourOutputMat, can.getFittedRect(), new Scalar(0, 255, 0));
            }
        }

        //drawing a midline to compare against heading
        Imgproc.line(finalContourOutputMat, new Point(CENTER_LINE, 0), new Point(CENTER_LINE, finalContourOutputMat.height()), new Scalar(255, 0, 0), 1);
        //Imgproc.line(finalContourOutputMat, new Point(RIGHT_THRESHOLD, 0), new Point(RIGHT_THRESHOLD, finalContourOutputMat.height()), new Scalar(255, 255, 255), 2);
        //Imgproc.line(finalContourOutputMat, new Point(LEFT_THRESHOLD, 0), new Point(LEFT_THRESHOLD, finalContourOutputMat.height()), new Scalar(255, 255, 255), 2);

        //This is the typical FTC position based test - not really useful for 6can
        if (largestX > 0 && largestX < LEFT_THRESHOLD) {
            lastPosition = Position.LEFT;
        } else if (largestX > LEFT_THRESHOLD && largestX < RIGHT_THRESHOLD) {
            lastPosition = Position.MIDDLE;
        } else if (largestX > RIGHT_THRESHOLD && largestX < cropOutput.width()) {
            lastPosition = Position.RIGHT;
        } else
            lastPosition = Position.NONE_FOUND;

        switch (VIEW_OPEN_CV_PIPELINE_STAGE) {
            case 0:
                dashboardMat = cropOutput;
                break;
            case 1:
                dashboardMat = normalizeOutput;
                break;
            case 2:
                dashboardMat = blurInput;
                break;
            case 3:
                dashboardMat = blurOutput;
                break;
            case 4:
                dashboardMat = hsvThresholdOutput;
                break;
            case 5:
                dashboardMat = findContoursOutputMat;
                break;
            case 6:
                dashboardMat = finalContourOutputMat;
                break;
            default:
                dashboardMat = input;
                break;
        }
        if (dashboardMat != null && !dashboardMat.empty()) {
            dashboardBitmap = Bitmap.createBitmap(dashboardMat.width(), dashboardMat.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(dashboardMat, dashboardBitmap);
        }

        return input;
    }

    public int[] getPosition() {
        return new int[]{largestX, largestY};
    }

    public synchronized List<Target> getDetectedCans() {
        return detectedCans;
    }

    public Bitmap getDashboardImage() {
        return dashboardBitmap;
    }

    /**
     * Normalizes or remaps the values of pixels in an image.
     *
     * @param input  The image on which to perform the Normalize.
     * @param type   The type of normalization.
     * @param a      The minimum value.
     * @param b      The maximum value.
     * @param output The image in which to store the output.
     */
    private void normalize(Mat input, int type, double a, double b, Mat output) {
        Core.normalize(input, output, a, b, type);
    }

    enum BlurType {
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            } else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            } else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            } else {
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
        int radius = (int) (doubleRadius + 0.5);
        int kernelSize;
        switch (type) {
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input, output, new Size(kernelSize, kernelSize), radius);
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
        contours.clear();
        int mode = Imgproc.RETR_LIST;
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    private RotatedRect getOrientation(List<MatOfPoint> contour) {
        RotatedRect rotatedRect;
        rotatedRect = Imgproc.fitEllipse((MatOfPoint2f) contour);
        return rotatedRect;
    }

    public Position getLastPosition() {
        return lastPosition;
    }

    public double[] getLargestCoordinate() {
        return new double[]{largestX, largestY};
    }

    public double getLargestAreaPixels() {
        return largestArea;
    }
}
