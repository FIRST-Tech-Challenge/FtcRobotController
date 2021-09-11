package org.firstinspires.ftc.teamcode.robots.goodBot.vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.robots.goodBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.BlobStats;
import org.opencv.android.Utils;
import org.opencv.core.Core;
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
import java.util.Iterator;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants.VISION_ONE_TO_FOUR_ASPECT;

public class Pipeline extends OpenCvPipeline {

    private List<BlobStats> blobs = new ArrayList<>();
    Mat cropOutput;
    Mat normalizeOutput = new Mat();
    Mat blurOutput = new Mat();
    Mat hsvThresholdOutput = new Mat();
    List<MatOfPoint> findContoursOutput = new ArrayList<>();
    List<MatOfPoint> filterContoursOutput = new ArrayList<>();
    public long cropTime, hsvTime, normalizeTime, blurTime, momentsTime, contourTime;
    private StackHeight lastStackHeight = StackHeight.NONE_FOUND;
    public double lastRatio = 0.0;
    private boolean enableDashboard;
    private FtcDashboard dashboard;

    public Pipeline(boolean enableDashboard) {
        this.enableDashboard = enableDashboard;
        if(enableDashboard)
            dashboard = FtcDashboard.getInstance();
    }

    public Mat processFrame(Mat input) {
        // Step crop:
        switch(Constants.ALLIANCE) {
            case RED:
                cropOutput = crop(input, new Point(Constants.TOP_LEFT_X_RED, Constants.TOP_LEFT_Y_RED), new Point(Constants.BOTTOM_RIGHT_X_RED, Constants.BOTTOM_RIGHT_Y_RED));
            case BLUE:
                cropOutput = crop(input, new Point(Constants.TOP_LEFT_X_BLUE, Constants.TOP_LEFT_Y_BLUE), new Point(Constants.BOTTOM_RIGHT_X_BLUE, Constants.BOTTOM_RIGHT_Y_BLUE));
        }

        cropTime = System.currentTimeMillis();

        // Step Normalize0:
        Mat normalizeInput = cropOutput;
        int normalizeType = Core.NORM_MINMAX;
        double normalizeAlpha = Constants.NORMALIZE_ALPHA;
        double normalizeBeta = Constants.NORMALIZE_BETA;
        normalize(normalizeInput, normalizeType, normalizeAlpha, normalizeBeta, normalizeOutput);
        normalizeTime = System.currentTimeMillis();

        // Step Blur0:
        Mat blurInput = normalizeOutput;
        BlurType blurType = BlurType.get("Median Blur");
        double blurRadius = Constants.BLUR_RADIUS;
        blur(blurInput, blurType, blurRadius, blurOutput);
        blurTime = System.currentTimeMillis();

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {Constants.HSV_THRESHOLD_HUE_MIN, Constants.HSV_THRESHOLD_HUE_MAX};
        double[] hsvThresholdSaturation = {Constants.HSV_THRESHOLD_SATURATION_MIN, Constants.HSV_THRESHOLD_SATURATION_MAX};
        double[] hsvThresholdValue = {Constants.HSV_THRESHOLD_VALUE_MIN, Constants.HSV_THRESHOLD_VALUE_MAX};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
        hsvTime = System.currentTimeMillis();


        // Step Find_Contours0:
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);
        contourTime = System.currentTimeMillis();


        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> each = findContoursOutput.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }

        // Filter contours by area and resize to fit the original image size
        blobs.clear();
        each = findContoursOutput.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            if (Imgproc.contourArea(contour) > Constants.MIN_CONTOUR_AREA * maxArea) {
                Core.multiply(contour, new Scalar(4,4), contour);
                filterContoursOutput.add(contour);
                Moments p = Imgproc.moments(contour, false);
                int x = (int) (p.get_m10() / p.get_m00());
                int y = (int) (p.get_m01() / p.get_m00());
                double area = Imgproc.contourArea(contour);
                Rect blobBox = Imgproc.boundingRect(contour);
                BlobStats blob = new BlobStats(p,x,y,blobBox.width,blobBox.height,area);
                blobs.add(blob); //put it in the List
//                Imgproc.circle(overlay, new Point(x, y), 5, new Scalar(0,255,0,255), -1);
            }
//            Imgproc.drawContours(overlay, filterContoursOutput, -1, new Scalar(0,255,0,255), 3);
        }
        momentsTime= System.currentTimeMillis();

        for(int i = 0; i < blobs.size(); i++) {
            if(blobs.get(i).area < Constants.MIN_BLOB_SIZE) {
                blobs.remove(i);
                i--;
            }
        }

        if(blobs.isEmpty())
            lastStackHeight =  StackHeight.ZERO;
        else {
            BlobStats largestBlob = new BlobStats(new Moments(), 0, 0, 0, 0, 0);
            Iterator<BlobStats> each2 = blobs.iterator();
            while(each2.hasNext()) {
                BlobStats blob = each2.next();
                if(blob.area > largestBlob.area)
                    largestBlob = blob;
            }
            lastRatio = ((double)largestBlob.width / (double) largestBlob.height);
            if(lastRatio > VISION_ONE_TO_FOUR_ASPECT)
                lastStackHeight = StackHeight.ONE;
            else
                lastStackHeight =  StackHeight.FOUR;
        }

        switch(Constants.visionView) {
            case 0:
                sendMatToDashboard(input);
                return input;
            case 1:
                sendMatToDashboard(cropOutput);
                return cropOutput;
            case 2:
                sendMatToDashboard(normalizeOutput);
                return normalizeOutput;
            case 3:
                sendMatToDashboard(blurOutput);
                return blurOutput;
            case 4:
                sendMatToDashboard(hsvThresholdOutput);
                return hsvThresholdOutput;
            default:
                sendMatToDashboard(input);
                return input;
        }
    }

    private void sendMatToDashboard(Mat mat) {
        Bitmap bm = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(mat, bm);
        dashboard.sendImage(bm);
    }

    private Mat crop(Mat image, Point topLeftCorner, Point bottomRightCorner) {
        Rect cropRect = new Rect(topLeftCorner, bottomRightCorner);
        return new Mat(image, cropRect);
    }

    /**
     * Normalizes or remaps the values of pixels in an image.
     * @param input The image on which to perform the Normalize.
     * @param type The type of normalization.
     * @param a The minimum value.
     * @param b The maximum value.
     * @param output The image in which to store the output.
     */
    private void normalize(Mat input, int type, double a, double b, Mat output) {
        Core.normalize(input, output, a, b, type);
    }

    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
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

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
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

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param output The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat output) {
        Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2HSV);
        Core.inRange(output, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), output);
    }

    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }

    public StackHeight getLastStackHeight() {
        return lastStackHeight;
    }
}
