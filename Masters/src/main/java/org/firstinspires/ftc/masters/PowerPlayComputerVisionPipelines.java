/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


public class PowerPlayComputerVisionPipelines {
    public enum PipePosition {
        LEFT8,
        LEFT7,
        LEFT6,
        LEFT5,
        LEFT4,
        LEFT3,
        LEFT2,
        LEFT1,
        CENTER,
        RIGHT1,
        RIGHT2,
        RIGHT3,
        RIGHT4,
        RIGHT5,
        RIGHT6,
        RIGHT7,
        RIGHT8,
        SHRUG_NOISES
    }

    public enum PipelineType {SLEEVE, PIPE, RED_CONE, BLUE_CONE}

    //    Declare webcam
    public OpenCvWebcam webcam;
    public OpenCvWebcam sleeveWebcam;

    //    Initial declaration of pipelines (One for each we use)
    public FrontPipeline frontPipeline;

    public SleevePipeline sleevePipeline;
    public PipeDetectionPipeline pipeDetectionPipeline;
    public BlueStackPipeline blueStackDetectionPipeline;
    Telemetry telemetry;
    boolean error = false;
    public PipelineType sleeveWebcamPipeline = PipelineType.SLEEVE;

//    Random useful tidbit of information, to access the camera stream, hit innit, then
//    press the ellipsis button on the top right of the screen, then select "Camera Stream".
//    Follow same path to turn it off


    public PowerPlayComputerVisionPipelines(HardwareMap hardwareMap, Telemetry telemetry) {

//        Get and store camera monitor view id.
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
        this.telemetry = telemetry;

        sleevePipeline = new SleevePipeline(telemetry);
        sleevePipeline.setPipelineType(PipelineType.SLEEVE);
        pipeDetectionPipeline = new PipeDetectionPipeline(telemetry);
        blueStackDetectionPipeline = new BlueStackPipeline(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        sleeveWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcamSleeve"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                telemetry.addData("webcam open", "yes");
                webcam.setPipeline(pipeDetectionPipeline);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Can't open camera");
                telemetry.update();
                error = true;
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        sleeveWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("sleeve webcam open", "yes");
                sleeveWebcam.setPipeline(sleevePipeline);
                sleeveWebcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Can't open sleeve camera");
                telemetry.update();
            }
        });

    }

    public boolean isError() {
        return error;
    }

    public void setSleevePipeline() {
        sleeveWebcamPipeline = PipelineType.SLEEVE;
        sleevePipeline.setPipelineType(sleeveWebcamPipeline);

    }

    public void setPipeDetectionFront() {
        sleeveWebcamPipeline = PipelineType.PIPE;
        sleevePipeline.setPipelineType(sleeveWebcamPipeline);
    }


    public void setBlueStackDetectionPipeline() {
        sleeveWebcamPipeline = PipelineType.BLUE_CONE;
    }

    public void stopCamera() {
        webcam.stopStreaming();
        webcam.setPipeline(new DoNothingPipeline());
    }

    public void stopSleeveCamera() {
        sleeveWebcam.stopStreaming();
        sleeveWebcam.setPipeline(new DoNothingPipeline());
    }

    //    The aforementioned pipeline. Aptly named.
    public static class DoNothingPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }

    public static class FrontPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }

    //Pipeline for finding sleeve color
    public static class SleevePipeline extends OpenCvPipeline {
        Telemetry telemetry;
        PipelineType pipelineType = PipelineType.SLEEVE;

        public SleevePipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        public void setPipelineType(PipelineType pipelineType) {
            this.pipelineType = pipelineType;
        }

        //    All possible regions to be detected in are stored in an enum
        public enum SleeveColor {
            RED,
            GREEN,
            GRAY,
            INDETERMINATE
        }


        /*
         * Some color constants (in RGB) used for displaying rectangles on the camera stream
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//        static final Scalar RED = new Scalar(255, 0, 0); e


        //        Sizes for subregions of the camera from which our data is extracted
        static final int SLEEVE_REGION_WIDTH = 32;
        static final int SLEEVE_REGION_HEIGHT = 50;

        /*
         * List for the storage of points, if you're only dealing with a few regions declare them all separately, the freight regions in the other pipeline
         * are done like this.
         */

        Point sleeveTopLeftPoint = new Point(304, 186);
        Point sleeveBottomRightPoint = new Point(sleeveTopLeftPoint.x + SLEEVE_REGION_WIDTH, sleeveTopLeftPoint.y + SLEEVE_REGION_HEIGHT);


        //        The thresholds to which the averages are compared.
        final int RED_SLEEVE_SIDE = 180;
        final int GRAY_SLEEVE_SIDE = 126;
        final int GREEN_SLEEVE_SIDE = 117;

        int distanceFromGreen;
        int distanceFromRed;
        int distanceFromGray;

        static final int PIPE_REGION_WIDTH = 40;
        static final int PIPE_REGION_HEIGHT = 100;

        ArrayList<Point> pipeTopLeftPoints = new ArrayList<>();
        ArrayList<Point> pipeBottomRightPoints = new ArrayList<>();

        final int PIPE_PRESENT_THRESHOLD = 155; //swicthing to 150 seems affected by the light was  174;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile PipePosition position = PipePosition.SHRUG_NOISES;
        public volatile SleeveColor color = SleeveColor.RED;
        /*
         * Empty matrices that data will be stored in
         */

        Mat SLEEVE_LAB = new Mat();
        Mat SLEEVE_A = new Mat();
        Mat SLEEVE_B = new Mat();

        Mat sleeve_aRegion;
        Mat sleeve_bRegion;

        int sleeve_aChannelAvg;
        int sleeve_bChannelAvg;

        Mat PIPE_LAB = new Mat();
        Mat PIPE_A = new Mat();
        Mat PIPE_B = new Mat();

        Mat pipe_aRegion;
        Mat pipe_bRegion;

        int pipe_aChannelAvg;
        int pipe_bChannelAvg;

        /*
         * This function takes the RGB frame, converts to LAB,
         * and extracts the A channel to the 'A' variable
         */
        void inputToLAB(Mat input) {

            if (pipelineType == PipelineType.SLEEVE) {
                Imgproc.cvtColor(input, SLEEVE_LAB, Imgproc.COLOR_RGB2Lab);
                Core.extractChannel(SLEEVE_LAB, SLEEVE_A, 1);
                Core.extractChannel(SLEEVE_LAB, SLEEVE_B, 2);
            }
            if (pipelineType == PipelineType.PIPE){
                Imgproc.cvtColor(input, PIPE_LAB, Imgproc.COLOR_RGB2Lab);
                Core.extractChannel(PIPE_LAB, PIPE_A, 1);
                Core.extractChannel(PIPE_LAB, PIPE_B, 2);
            }
        }

        //        Done in innit, this was a more complicated formation of subregions for detecting the duck, but essentially
//        just assign the top left and bottom right points for each region you desire.
        @Override
        public void init(Mat firstFrame) {
                pipeTopLeftPoints.add(new Point(140, 30));
                pipeBottomRightPoints.add(new Point(pipeTopLeftPoints.get(0).x + PIPE_REGION_WIDTH, pipeTopLeftPoints.get(0).y + PIPE_REGION_HEIGHT));

                for (int i = 1; i < 17; i++) {
                    pipeTopLeftPoints.add(new Point(pipeTopLeftPoints.get(i - 1).x + PIPE_REGION_WIDTH / 2, pipeTopLeftPoints.get(0).y));
                    pipeBottomRightPoints.add(new Point(pipeTopLeftPoints.get(i).x + PIPE_REGION_WIDTH, pipeTopLeftPoints.get(i).y + PIPE_REGION_HEIGHT));
                }

            inputToLAB(firstFrame);
        }

        //        Process frame, takes a matrix input and processes it. I still have no idea WHERE this is called, but it is absolutely essential to CV functioning
        @Override
        public Mat processFrame(Mat input) {
            if (this.pipelineType == PipelineType.SLEEVE) {
                return processSleeve(input);
            }
            if (this.pipelineType == PipelineType.PIPE) {
                return processPipe(input);
            }
            if (this.pipelineType == PipelineType.BLUE_CONE) {

            }
            return input;

        }

        public Mat processSleeve(Mat input) {
            inputToLAB(input);

//            Declare regions
            sleeve_aRegion = SLEEVE_A.submat(new Rect(sleeveTopLeftPoint, sleeveBottomRightPoint));
            sleeve_bRegion = SLEEVE_B.submat(new Rect(sleeveTopLeftPoint, sleeveBottomRightPoint));

            sleeve_aChannelAvg = (int) Core.mean(sleeve_aRegion).val[0];
            sleeve_bChannelAvg = (int) Core.mean(sleeve_bRegion).val[0];


//              This is what displays the rectangle to the camera stream on the drive hub
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    sleeveTopLeftPoint, // First point which defines the rectangle
                    sleeveBottomRightPoint, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

//            Display telemetry
            telemetry.addData("A Average",sleeve_aChannelAvg);
            telemetry.addData("B Average", sleeve_bChannelAvg);

            distanceFromGreen = Math.abs(sleeve_aChannelAvg - GREEN_SLEEVE_SIDE);
            distanceFromRed = Math.abs(sleeve_aChannelAvg - RED_SLEEVE_SIDE);
            distanceFromGray = Math.abs(sleeve_aChannelAvg - GRAY_SLEEVE_SIDE);


            telemetry.addData("distanceFromGreen", distanceFromGreen);
            telemetry.addData("distanceFromRed", distanceFromRed);
            telemetry.addData("distanceFromGray", distanceFromGray);


            if (distanceFromGreen < distanceFromRed && distanceFromGreen < distanceFromGray) {
                color = SleeveColor.GREEN;
            } else if (distanceFromRed < distanceFromGreen && distanceFromRed < distanceFromGray) {
                color = SleeveColor.RED;
            } else if (distanceFromGray < distanceFromRed) {
                color = SleeveColor.GRAY;
            }

            telemetry.addData("Color detected", color);
            telemetry.update();

            SLEEVE_LAB.release();
            SLEEVE_A.release();
            SLEEVE_B.release();
            sleeve_aRegion.release();
            sleeve_aRegion.release();

            return input;
        }

        public Mat processPipe(Mat input) {
            inputToLAB(input);

//            Declare list of regions
            ArrayList<Mat> region = new ArrayList<>();

//            Put the necessary data from the frame to each region
            for (int i = 0; i < 17; i++) {
                region.add(PIPE_B.submat(new Rect(pipeTopLeftPoints.get(i), pipeBottomRightPoints.get(i))));
            }

            ArrayList<Integer> regionAvgs = new ArrayList<>();

            for (int i = 0; i < 17; i++) {
                regionAvgs.add((int) Core.mean(region.get(i)).val[0]);
            }

//              This is what displays the rectangles to the camera stream on the drive hub
            for (int i = 0; i < 17; i++) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        pipeTopLeftPoints.get(i), // First point which defines the rectangle
                        pipeBottomRightPoints.get(i), // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines

            }

//            The next several lines determine which region has the highest B average.
            int indexOfMaximumBAvg = 0;

            for (int i = 0; i < 17; i++) {
                if (regionAvgs.get(i) >= regionAvgs.get(indexOfMaximumBAvg)) {
                    indexOfMaximumBAvg = i;
                }
            }

//            Display telemetry
            telemetry.addData("B Averages", regionAvgs);
            telemetry.addData("Index of highest likelihood.", indexOfMaximumBAvg);

//            Fix indexes (Regions are stacked 17 to a row)
            if (regionAvgs.get(indexOfMaximumBAvg) >= PIPE_PRESENT_THRESHOLD) {
                position = PipePosition.values()[indexOfMaximumBAvg];
            } else {
                position = PipePosition.SHRUG_NOISES; // Default enum result. Aptly named
            }

            for (Mat mat: region){
                mat.release();
            }

            telemetry.addData("Position", position);
            telemetry.update();

            return input;
        }
    }

    public static class PipeDetectionPipeline extends OpenCvPipeline {
        Telemetry telemetry;

        public PipeDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        //    All possible regions to be detected in are stored in an enum


        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 100;

        ArrayList<Point> topLeftPoints = new ArrayList<>();
        ArrayList<Point> bottomRightPoints = new ArrayList<>();

        final int PIPE_PRESENT_THRESHOLD = 155;

        Mat LAB = new Mat();
        Mat A = new Mat();
        Mat B = new Mat();

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile PipePosition position = PipePosition.SHRUG_NOISES;

        void inputToLAB(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 1);
            Core.extractChannel(LAB, B, 2);
        }

        //        Done in innit, this was a more complicated formation of subregions for detecting the duck, but essentially
//        just assign the top left and bottom right points for each region you desire.
        @Override
        public void init(Mat firstFrame) {

            topLeftPoints.add(new Point(140, 30));
            bottomRightPoints.add(new Point(topLeftPoints.get(0).x + REGION_WIDTH, topLeftPoints.get(0).y + REGION_HEIGHT));

            for (int i = 1; i < 17; i++) {
                topLeftPoints.add(new Point(topLeftPoints.get(i - 1).x + REGION_WIDTH / 2, topLeftPoints.get(0).y));
                bottomRightPoints.add(new Point(topLeftPoints.get(i).x + REGION_WIDTH, topLeftPoints.get(i).y + REGION_HEIGHT));
            }

            inputToLAB(firstFrame);


        }

        //        Process frame, takes a matrix input and processes it. I still have no idea WHERE this is called, but it is absolutely essential to CV functioning
        @Override
        public Mat processFrame(Mat input) {
            inputToLAB(input);

//            Declare list of regions
            ArrayList<Mat> region = new ArrayList<>();


//            Put the necessary data from the frame to each region
            for (int i = 0; i < 17; i++) {
                region.add(B.submat(new Rect(topLeftPoints.get(i), bottomRightPoints.get(i))));
            }

            ArrayList<Integer> regionAvgs = new ArrayList<>();

            for (int i = 0; i < 17; i++) {
                regionAvgs.add((int) Core.mean(region.get(i)).val[0]);
            }

//              This is what displays the rectangles to the camera stream on the drive hub
            for (int i = 0; i < 17; i++) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        topLeftPoints.get(i), // First point which defines the rectangle
                        bottomRightPoints.get(i), // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines

            }

//            The next several lines determine which region has the highest B average.
            int indexOfMaximumBAvg = 0;

            for (int i = 0; i < 17; i++) {
                if (regionAvgs.get(i) >= regionAvgs.get(indexOfMaximumBAvg)) {
                    indexOfMaximumBAvg = i;
                }
            }

//            Display telemetry
            telemetry.addData("B Averages", regionAvgs);
            telemetry.addData("Index of highest likelihood.", indexOfMaximumBAvg);

//            Fix indexes (Regions are stacked 17 to a row)
            if (regionAvgs.get(indexOfMaximumBAvg) >= PIPE_PRESENT_THRESHOLD) {
                position = PipePosition.values()[indexOfMaximumBAvg];
            } else {
                position = PipePosition.SHRUG_NOISES; // Default enum result. Aptly named
            }


            telemetry.addData("Position", position);
            telemetry.update();

            return input;
        }
    }

    public static class BlueStackPipeline extends OpenCvPipeline {
        Telemetry telemetry;

        public BlueStackPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        //    All possible regions to be detected in are stored in an enum
        public enum StackPosition {
            LEFT6,
            LEFT5,
            LEFT4,
            LEFT3,
            LEFT2,
            LEFT1,
            CENTER,
            RIGHT1,
            RIGHT2,
            RIGHT3,
            RIGHT4,
            RIGHT5,
            RIGHT6,
            SHRUG_NOISES
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final int REGION_WIDTH = 102;
        static final int REGION_HEIGHT = 80;

        ArrayList<Point> topLeftPoints = new ArrayList<>();
        ArrayList<Point> bottomRightPoints = new ArrayList<>();

        final int STACK_PRESENT_THRESHOLD = 90;

        Mat LAB = new Mat();
        Mat A = new Mat();
        Mat B = new Mat();

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile StackPosition position = StackPosition.SHRUG_NOISES;

        void inputToLAB(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 1);
            Core.extractChannel(LAB, B, 2);
        }

        //        Done in innit, this was a more complicated formation of subregions for detecting the duck, but essentially
//        just assign the top left and bottom right points for each region you desire.
        @Override
        public void init(Mat firstFrame) {

            topLeftPoints.add(new Point(65, 140));
            bottomRightPoints.add(new Point(topLeftPoints.get(0).x + REGION_WIDTH, topLeftPoints.get(0).y + REGION_HEIGHT));

            for (int i = 1; i < 17; i++) {
                topLeftPoints.add(new Point(topLeftPoints.get(i - 1).x + REGION_WIDTH / 3, topLeftPoints.get(0).y));
                bottomRightPoints.add(new Point(topLeftPoints.get(i).x + REGION_WIDTH, topLeftPoints.get(i).y + REGION_HEIGHT));
            }

            inputToLAB(firstFrame);

        }

        //        Process frame, takes a matrix input and processes it. I still have no idea WHERE this is called, but it is absolutely essential to CV functioning
        @Override
        public Mat processFrame(Mat input) {
            inputToLAB(input);

//            Declare list of regions
            ArrayList<Mat> region = new ArrayList<>();


//            Put the necessary data from the frame to each region
            for (int i = 0; i < 13; i++) {
                region.add(B.submat(new Rect(topLeftPoints.get(i), bottomRightPoints.get(i))));
            }

            ArrayList<Integer> regionAvgs = new ArrayList<>();

            for (int i = 0; i < 13; i++) {
                regionAvgs.add((int) Core.mean(region.get(i)).val[0]);
            }

//              This is what displays the rectangles to the camera stream on the drive hub
            for (int i = 0; i < 13; i++) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        topLeftPoints.get(i), // First point which defines the rectangle
                        bottomRightPoints.get(i), // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines

            }

//            The next several lines determine which region has the highest B average.
            int indexOfMaximumBAvg = 0;

            for (int i = 0; i < 13; i++) {
                if (regionAvgs.get(i) >= regionAvgs.get(indexOfMaximumBAvg)) {
                    indexOfMaximumBAvg = i;
                }
            }

//            Display telemetry
            telemetry.addData("B Averages", regionAvgs);
            telemetry.addData("Index of highest likelihood.", indexOfMaximumBAvg);

//            Fix indexes (Regions are stacked 17 to a row)
            if (regionAvgs.get(indexOfMaximumBAvg) <= STACK_PRESENT_THRESHOLD) {
                position = StackPosition.values()[indexOfMaximumBAvg];
            } else {
                position = StackPosition.SHRUG_NOISES; // Default enum result. Aptly named
            }


            telemetry.addData("Position", position);
            telemetry.update();


            telemetry.update();

            return input;
        }
    }
}