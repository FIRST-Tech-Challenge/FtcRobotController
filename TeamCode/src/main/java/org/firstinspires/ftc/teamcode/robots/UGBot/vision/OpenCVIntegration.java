package org.firstinspires.ftc.teamcode.robots.UGBot.vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.util.BlobStats;
import org.firstinspires.ftc.teamcode.util.VisionUtils;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.opencv.android.Utils;
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

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.BlockingQueue;

public class OpenCVIntegration implements VisionProvider {
    private VuforiaLocalizer vuforia;
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> q;
    private boolean enableDashboard;
    private List<BlobStats> blobs = new ArrayList<>();
    private FtcDashboard dashboard;


    private void initVuforia(HardwareMap hardwareMap, Viewpoint viewpoint) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        if (viewpoint == Viewpoint.BACK)
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        else if (viewpoint == Viewpoint.WEBCAM)
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        else
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    @Override
    public void initializeVision(HardwareMap hardwareMap, Viewpoint viewpoint, boolean enableDashboard) {
        initVuforia(hardwareMap, viewpoint);
        this.enableDashboard = enableDashboard;
        if(enableDashboard)
            dashboard = FtcDashboard.getInstance();
        q = vuforia.getFrameQueue();
    }

    @Override
    public void shutdownVision() {}

    @Override
    public StackHeight detect() {
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("q size", q.size());
//        dashboard.sendTelemetryPacket(packet);
        if (q.isEmpty())
            return StackHeight.HOLD_STATE;
        VuforiaLocalizer.CloseableFrame frame;
        try {
            frame = q.take();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Image img = VisionUtils.getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());

        Mat overlay = process(VisionUtils.bitmapToMat(bm, CvType.CV_8UC3));

        if(enableDashboard) {
            Bitmap overlayBitmap = Bitmap.createBitmap(overlay.width(), overlay.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(overlay, overlayBitmap);
            dashboard.sendImage(overlayBitmap);
            overlay.release();
        }

        if(blobs.isEmpty())
            return StackHeight.ZERO;
        else {
            BlobStats largestBlob = new BlobStats(new Moments(), 0, 0, 0, 0, 0);
            Iterator<BlobStats> each = blobs.iterator();
            while(each.hasNext()) {
                BlobStats blob = each.next();
                if(blob.area > largestBlob.area)
                    largestBlob = blob;
            }
            if((largestBlob.width / (double) largestBlob.height) > 2)
                return StackHeight.ONE;
            return StackHeight.FOUR;
        }
    }

    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public Mat process(Mat source0) {
        Mat cropOutput;
        Mat normalizeOutput = new Mat();
        Mat blurOutput = new Mat();
        Mat hsvThresholdOutput = new Mat();
        List<MatOfPoint> findContoursOutput = new ArrayList<>();
        List<MatOfPoint> filterContoursOutput = new ArrayList<>();

        // Step crop:
        cropOutput = crop(source0, new Point(Constants.TOP_LEFT_X, Constants.TOP_LEFT_Y), new Point(Constants.BOTTOM_RIGHT_X, Constants.BOTTOM_RIGHT_Y));

        // Step Normalize0:
        Mat normalizeInput = cropOutput;
        int normalizeType = Core.NORM_MINMAX;
        double normalizeAlpha = Constants.NORMALIZE_ALPHA;
        double normalizeBeta = Constants.NORMALIZE_BETA;
        normalize(normalizeInput, normalizeType, normalizeAlpha, normalizeBeta, normalizeOutput);

        // Step Blur0:
        Mat blurInput = normalizeOutput;
        BlurType blurType = BlurType.get("Gaussian Blur");
            double blurRadius = Constants.BLUR_RADIUS;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step HSV_Threshold0:
        Mat hsvThresholdInput = blurOutput;
        double[] hsvThresholdHue = {Constants.HSV_THRESHOLD_HUE_MIN, Constants.HSV_THRESHOLD_HUE_MAX};
        double[] hsvThresholdSaturation = {Constants.HSV_THRESHOLD_SATURATION_MIN, Constants.HSV_THRESHOLD_SATURATION_MAX};
        double[] hsvThresholdValue = {Constants.HSV_THRESHOLD_VALUE_MIN, Constants.HSV_THRESHOLD_VALUE_MAX};
        hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step Find_Contours0:
        Mat findContoursInput = hsvThresholdOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

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
        Mat overlay = source0.clone();
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
                Imgproc.circle(overlay, new Point(x, y), 5, new Scalar(0,255,0,255), -1);
            }
            Imgproc.drawContours(overlay, filterContoursOutput, -1, new Scalar(0,255,0,255), 3);
        }
        return overlay;
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

    @Override
    public void reset() {}
}
