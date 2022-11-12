/* Author: Kai Vernooy
 */

package org.firstinspires.ftc.teamcode;

import com.google.gson.JsonArray;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.*;

import androidx.annotation.Nullable;
import android.os.Environment;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import java.util.*;

import org.opencv.android.Utils;
import android.graphics.BitmapFactory;
import android.graphics.Bitmap;

import org.opencv.core.*;

import org.opencv.features2d.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.Calib3d;

import org.openftc.easyopencv.*;


/** Managing class for opening cameras, attaching pipelines, and beginning streaming.
 */
public class ComputerVision {

    public OpenCvCamera camera;
    public OpenCvPipeline pipeline;

    public static String DataDir = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/cvdata";


    ComputerVision(HardwareMap hardwareMap, OpenCvPipeline pipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        this.pipeline = pipeline;
    }


    /** Begins continuous frame acquisition and image processing.
     */
    public void startStreaming() {
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }


            // TODO: responsible error handling
            @Override
            public void onError(int errorCode) {}
        });
    }

    public void stopStreaming() {
        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {}
        });
    }
}




/** Contains all image processing done for scanning the barcode and getting position.
 * This will
 */
class AutonPipeline extends OpenCvPipeline {
    final private Robot robot;
    final private Telemetry telemetry;
    final private Mat output;  // Frame to be displayed on the phone
    final private RobotManager.AllianceColor allianceColor;

    boolean first = true;


    AutonPipeline(Robot robot, Telemetry telemetry, RobotManager.AllianceColor allianceColor) {
        super();
        this.robot = robot;

        // TODO: there might be a cleaner way to default-initialize these
        output = new Mat();


        // Initialize barcode data
        barcodeHsv = new Mat();
        barcodeTapeRegions = new Mat();
        barcodeTapeLabels = new Mat();
        barcodeTapeStats = new Mat();
        barcodeTapeCentroids = new Mat();
        barcodeCapRegions = new Mat();
        barcodeCapLabels = new Mat();
        barcodeCapStats = new Mat();
        barcodeCapCentroids = new Mat();
        barcodeTapeRegionsBlue = new Mat();
        barcodeTapeRegionsRed1 = new Mat();
        barcodeTapeRegionsRed2 = new Mat();


        this.telemetry = telemetry;
        this.allianceColor = allianceColor;
    }


    /**
     * The main pipeline method, called whenever a frame is received. It will execute all necessary CV tasks, such as localization and barcode scanning
     *
     * @param input The current frame read from the attached camera.
     *              NOTE: the camera will be mounted in landscape, so make sure to flip x/y coords
     * @return An output frame to be displayed on the phone
     */
    @Override
    public Mat processFrame(Mat input) {
//        return input;
////        if (first) {
//            saveMatToDiskFullPath(input, ComputerVision.DataDir + "/firstimage.png");
//            first = false;
//        }


        // Check if a barcode scan has been requested
        if (robot.barcodeScanState == Robot.BarcodeScanState.SCAN) {
            // Scan the barcode
            input.copyTo(output);
            Robot.BarcodeScanResult result = processBarcodeFrame(input, output);

            if (robot.numBarcodeAttempts % 5 == 0) {
                saveMatToDiskFullPath(output, ComputerVision.DataDir + "/barcodeImage" + robot.numBarcodeAttempts + ".png");
            }

            // Increment the barcode result in the frequency counter and find the max value in that map
            int freq = robot.barcodeScanResultMap.get(result);
            robot.barcodeScanResultMap.put(result, freq + 1);

            Map.Entry<Robot.BarcodeScanResult, Integer> max = Collections.max(robot.barcodeScanResultMap.entrySet(), Comparator.comparingInt(Map.Entry::getValue));
            robot.numBarcodeAttempts++;

            if (robot.numBarcodeAttempts >= Robot.MAX_BARCODE_ATTEMPTS || max.getValue() >= Robot.MIN_BARCODE_REPEAT) {
                Map<Robot.BarcodeScanResult, Integer> fullResultMap = robot.barcodeScanResultMap;

                // Ensure that we don't end up with an invalid state as the most frequent. This will modify the map, so save a copy first.
                while (max.getKey() == Robot.BarcodeScanResult.WRONG_CAPS || max.getKey() == Robot.BarcodeScanResult.WRONG_TAPE) {
                    robot.barcodeScanResultMap.remove(max.getKey());
                    max = Collections.max(robot.barcodeScanResultMap.entrySet(), Comparator.comparingInt(Map.Entry::getValue));
                }

                robot.barcodeScanResult = max.getKey();
                robot.barcodeScanState = Robot.BarcodeScanState.CHECK_SCAN;
                robot.barcodeScanResultMap = fullResultMap;
            }
            else {
                return input;  // We have more iterations of barcode scanning to do, so we needn't spend time on positioning
            }
        }

//        Position currentPosition = processPositioningFrame(input, output);
//        if (currentPosition != null) robot.positionManager.updateCvPosition(currentPosition);

        return input;
    }


    // BARCODE SCANNING
    // =================

    // Single-time allocated mats that will hold frame processing data
    final private Mat barcodeHsv;
    final private Mat barcodeCapRegions, barcodeTapeRegions;
    final private Mat barcodeTapeRegionsBlue, barcodeTapeRegionsRed1, barcodeTapeRegionsRed2;
    final private Mat barcodeTapeLabels, barcodeTapeStats, barcodeTapeCentroids, barcodeCapLabels, barcodeCapStats, barcodeCapCentroids;

    // The Region of Interest that contains all the barcode elements and the least non-floor background possible
    final static int BarcodeCropLeft = 150;
    final static int BarcodeCropTop = 180;
    final static int BarcodeCropRight = 30;

    // Define HSV scalars that represent ranges of color to be selected from the barcode image
    final static Scalar[] BarcodeCapRange      = {new Scalar(23, 60, 50), new Scalar(78, 255, 255)};
    final static Scalar[] BarcodeTapeRangeBlue = {new Scalar(100, 100, 50), new Scalar(120, 255, 255)};
    final static Scalar[] BarcodeTapeRangeRed1 = {new Scalar(170, 100, 50), new Scalar(180, 255, 255)};
    final static Scalar[] BarcodeTapeRangeRed2 = {new Scalar(0,   100, 50), new Scalar(10,  255, 255)};

    static final Size NoiseSize = new Size(5, 5);

    /** Isolates the sections of an image in a given HSV range and removes noise, to find large solid-color areas
     * @param hsv The input image to be isolated, in HSV color format
     * @param out The image in which the detected areas will be stored
     * @param a HSV color in Scalar format that represents the lower bound of the area to be isolated
     * @param b HSV color in Scalar format that represents the upper bound of the area to be isolated
     * NOTE: OpenCV represents hue from 0-180
     */
    private static void IsolateBarcodeRange(Mat hsv, Mat out, Scalar a, Scalar b) {
        Core.inRange(hsv, a, b, out);

        Imgproc.morphologyEx(out, out, Imgproc.MORPH_CLOSE, Mat.ones(NoiseSize, CvType.CV_32F));
        Imgproc.morphologyEx(out, out, Imgproc.MORPH_OPEN, Mat.ones(NoiseSize, CvType.CV_32F));
    }


    private static class BarcodeCentroid implements Comparable<BarcodeCentroid> {
        BarcodeCentroid(int index, double centroidX) {
            this.index = index;
            this.centroidX = centroidX;
        }

        @Override
        public int compareTo(BarcodeCentroid other) {
            return Double.compare(this.centroidX, other.centroidX);
        }

        public int index;
        public double centroidX;
    };


    public static int[] BarcodeFlags = {235, 415, 570};


    /**
     * @param input The current frame containing the barcode to be scanned
     * @return an integer in the interval [-1, 2], where -1 denotes no result, and 0-2 represent positions (in screen space) of the object of interest
     */
    private Robot.BarcodeScanResult processBarcodeFrame(Mat input, Mat output) {
        Mat frame = input.submat(new Rect(0, BarcodeCropLeft, BarcodeCropTop, input.rows() - BarcodeCropLeft));

        // Convert input image to HSV space and perform basic blur
        Imgproc.cvtColor(frame, barcodeHsv, Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(barcodeHsv, barcodeHsv, new Size(7, 7), 5);

        // Do HSV thresholding to identify the barcode tape as well as the shipping element
        IsolateBarcodeRange(barcodeHsv, barcodeCapRegions, BarcodeCapRange[0], BarcodeCapRange[1]);

        // HSV thresholding for barcode tape isolation
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsRed1, BarcodeTapeRangeRed1[0], BarcodeTapeRangeRed1[1]);
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsRed2, BarcodeTapeRangeRed2[0], BarcodeTapeRangeRed2[1]);
        IsolateBarcodeRange(barcodeHsv, barcodeTapeRegionsBlue, BarcodeTapeRangeBlue[0], BarcodeTapeRangeBlue[1]);

        Core.bitwise_or(barcodeTapeRegionsRed1, barcodeTapeRegionsRed2, barcodeTapeRegions);
        Core.bitwise_or(barcodeTapeRegionsBlue, barcodeTapeRegions, barcodeTapeRegions);


//         Visualize the detected areas with appropriately colored outlines
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(barcodeCapRegions, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Draw the detected areas to the output for visualization
        // TODO: Get rid of this for competition


        input.copyTo(output);

        for (int idx = 0; idx < contours.size(); idx++) {
            Imgproc.drawContours(output, contours, idx, new Scalar(255, 255, 0), 6);
        }

        contours.clear();
        Imgproc.findContours(barcodeTapeRegions, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (int idx = 0; idx < contours.size(); idx++) {
            Imgproc.drawContours(output, contours, idx, new Scalar(255, 0, 0), 6);
        }

//
//        Imgproc.line(output, new org.opencv.core.Point(0, BarcodeFlags[0]), new org.opencv.core.Point(input.rows(), BarcodeFlags[0]), new Scalar(255, 0, 0));
//        Imgproc.line(output, new org.opencv.core.Point(0, BarcodeFlags[1]), new org.opencv.core.Point(input.rows(), BarcodeFlags[1]), new Scalar(0, 255, 0));
//        Imgproc.line(output, new org.opencv.core.Point(0, BarcodeFlags[2]), new org.opencv.core.Point(input.rows(), BarcodeFlags[2]), new Scalar(0, 0, 255));
//
//
//        return Robot.BarcodeScanResult.CENTER;        return Robot.BarcodeScanResult.RIGHT;

//
        // Determine the centroids of the tape regions
        int tapeComponentsCount = Imgproc.connectedComponentsWithStats(barcodeTapeRegions, barcodeTapeLabels, barcodeTapeStats, barcodeTapeCentroids, 8);

        // Get a sorted list of centroids
        List<BarcodeCentroid> tapeCentroidsList = new ArrayList<BarcodeCentroid>();
        for (int i = 1; i < tapeComponentsCount; i++) tapeCentroidsList.add(new BarcodeCentroid(i, barcodeTapeCentroids.at(double.class, i, 1).getV()));


        if (tapeCentroidsList.size() > 3) {
            Collections.sort(tapeCentroidsList);

            if (allianceColor == RobotManager.AllianceColor.RED) Collections.reverse(tapeCentroidsList);
            tapeCentroidsList = tapeCentroidsList.subList(0, 2);

            if (allianceColor == RobotManager.AllianceColor.RED) Collections.reverse(tapeCentroidsList);
            tapeComponentsCount = 3;
        }


        // Determine the centroid of the cap region
        int capComponentsCount = Imgproc.connectedComponentsWithStats(barcodeCapRegions, barcodeCapLabels, barcodeCapStats, barcodeCapCentroids, 8);
        if (capComponentsCount != 2) return Robot.BarcodeScanResult.WRONG_CAPS;

        double capCentroidX = barcodeCapCentroids.at(double.class, 1, 1).getV();

        // For now, we'll make sure that we're identifying only two non-cap tapes.
        if (tapeComponentsCount != 3) {
            int closestIndex = 0;
            double minDistance = input.rows();

            for (int i = 0; i < BarcodeFlags.length; i++) {
                double dist = Math.abs(capCentroidX - BarcodeFlags[i]);
                if (dist < minDistance) {
                    minDistance = dist;
                    closestIndex = i;
                }
            }

            return Robot.BarcodeScanResult.ResultFromValue(closestIndex);
        }

        if (capCentroidX < tapeCentroidsList.get(0).centroidX) return Robot.BarcodeScanResult.LEFT;
        else if (capCentroidX < tapeCentroidsList.get(1).centroidX) return Robot.BarcodeScanResult.CENTER;
        return Robot.BarcodeScanResult.RIGHT;
    }


    @Override
    public void onViewportTapped() {
//        camera.pauseViewport();
    }
}