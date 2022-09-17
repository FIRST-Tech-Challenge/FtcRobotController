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


public class ComputerVisionCodeAlong {

//    Declare webcam
    public OpenCvWebcam webcam;

//    Initial declaration of pipeline
    public ElementDetectionPipeline pipeline;

    public ComputerVisionCodeAlong(HardwareMap hardwareMap, Telemetry telemetry){

//        Get and store camera monitor view id. (If using multiple cameras.
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"));
        pipeline = new ElementDetectionPipeline(telemetry);

        webcam.setMillisecondsPermissionTimeout(2500);


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
                telemetry.update();
                webcam.setPipeline(pipeline);
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Can't open camera");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

//    Function for stopping cameras and switching to a pipeline that we don't waste resources.

    public void stopCamera(){
        webcam.stopStreaming();
        webcam.setPipeline(new DoNothingPipeline());
    }


//    The aforementioned pipeline. Aptly named.
    public static class DoNothingPipeline extends OpenCvPipeline{

        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }



//    Element detection pipeline
    public static class ElementDetectionPipeline extends OpenCvPipeline {

        public static int avg;
        Telemetry telemetry;

        public ElementDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        public enum ElementPosition {
            PRESENT,
            NOT_PRESENT
        }

        /*
         * Some color constants for display
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

//        Region sizes for freight submat, simplifies things if you're working with multiple regions.
        static final int REGION_WIDTH = 80;
        static final int REGION_HEIGHT = 100;

//        Keep in mind current dimensions are 640 x 360

        /*
        * Top left anchor points for region       */
        static final Point REGION_TOP_LEFT_ANCHOR_POINT = new Point(280, 80);

        Point region_pointA = new Point(
                REGION_TOP_LEFT_ANCHOR_POINT.x,
                REGION_TOP_LEFT_ANCHOR_POINT.y);
        Point region_pointB = new Point(
                REGION_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    //      Threshold for freight presence
        final int ELEMENT_PRESENT_THRESHOLD = 108;


        /*
         * Working variables
         */
        Mat region;

        Mat LAB = new Mat();

        Mat A = new Mat();
        Mat B = new Mat();


        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile ElementPosition position = ElementPosition.NOT_PRESENT;


        /*
         * This function takes the RGB frame, converts to LAB,
         * and extracts both channels used.*/

        void inputToLAB(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 1);
            Core.extractChannel(LAB, B, 2);
        }

        @Override
        public void init(Mat firstFrame) {

            inputToLAB(firstFrame);

            region = A.submat(new Rect(region_pointA, region_pointB));
        }


        @Override
        public Mat processFrame(Mat input) {
            inputToLAB(input);
            region = A.submat(new Rect(region_pointA, region_pointB));

            avg = (int) Core.mean(region).val[0];


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region_pointA, // First point which defines the rectangle
                    region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            if (avg <= ELEMENT_PRESENT_THRESHOLD) {
                position = ElementPosition.PRESENT;
            } else {
                position = ElementPosition.NOT_PRESENT;
            }

            telemetry.addData("Analysis", avg);
            telemetry.addData("Position", position);
            telemetry.update();

            return input;
        }
    }
}