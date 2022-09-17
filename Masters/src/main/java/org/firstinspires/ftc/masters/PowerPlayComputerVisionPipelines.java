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


public class PowerPlayComputerVisionPipelines {

//    Declare webcam
    public OpenCvWebcam webcam;

//    Initial declaration of pipelines (One for each we use)
    public SleevePipeline sleevePipeline;

//    Random useful tidbit of information, to access the camera stream, hit innit, then
//    press the ellipsis button on the top right of the screen, then select "Camera Stream".
//    Follow same path to turn it off


    public PowerPlayComputerVisionPipelines(HardwareMap hardwareMap, Telemetry telemetry){

//        Get and store camera monitor view id.
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        sleevePipeline = new SleevePipeline(telemetry);

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
                webcam.setPipeline(sleevePipeline);
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

//Pipeline for finding duck
    public static class SleevePipeline extends OpenCvPipeline{
        Telemetry telemetry;
        public SleevePipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

//    All possible regions to be detected in are stored in an enum
        public enum SleeveColor {
            RED,
            GREEN,
            YELLOW,
            INDETERMINATE
        }


        /*
         * Some color constants (in RGB) used for displaying rectangles on the camera stream
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//        static final Scalar RED = new Scalar(255, 0, 0); e


//        Sizes for subregions of the camera from which our data is extracted
        static final int REGION_WIDTH = 75;
        static final int REGION_HEIGHT = 150;

        /*
         * List for the storage of points, if you're only dealing with a few regions declare them all separately, the freight regions in the other pipeline
         * are done like this.
         */

        Point topLeftPoint = new Point(285,105);
        Point bottomRightPoint = new Point(topLeftPoint.x+REGION_WIDTH, topLeftPoint.y+REGION_HEIGHT);


//        The thresholds to which the averages are compared.
        final int RED_SLEEVE_SIDE = 140;
        final int YELLOW_SLEEVE_SIDE = 140;
        final int GREEN_SLEEVE_SIDE = 140;



        /*
         * Empty matrices that data will be stored in
         */

        Mat LAB = new Mat();
        Mat A = new Mat();
        Mat B = new Mat();

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SleeveColor color = SleeveColor.RED;

        /*
         * This function takes the RGB frame, converts to LAB,
         * and extracts the A channel to the 'A' variable
         */
        void inputToLAB(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 1);
            Core.extractChannel(LAB, B, 2);
        }

//        Done in innit, this was a more complicated formation of subregions for detecting the duck, but essentially
//        just assign the top left and bottom right points for each region you desire.
        @Override
        public void init(Mat firstFrame) {
            inputToLAB(firstFrame);
        }

//        Process frame, takes a matrix input and processes it. I still have no idea WHERE this is called, but it is absolutely essential to CV functioning
        @Override
        public Mat processFrame(Mat input) {
            inputToLAB(input);

//            Declare regions
            Mat aRegion = A.submat(new Rect(topLeftPoint, bottomRightPoint));
            Mat bRegion = B.submat(new Rect(topLeftPoint, bottomRightPoint));


            int aChannelAvg = (int) Core.mean(aRegion).val[0];
            int bChannelAvg = (int) Core.mean(bRegion).val[0];


//              This is what displays the rectangle to the camera stream on the drive hub
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    topLeftPoint, // First point which defines the rectangle
                    bottomRightPoint, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

//            Display telemetry
            telemetry.addData("A Average", aChannelAvg);
            telemetry.addData("B Average", bChannelAvg);

//
            int distanceFromGreen = aChannelAvg - GREEN_SLEEVE_SIDE;
            int distanceFromRed = aChannelAvg - RED_SLEEVE_SIDE;
            int distanceFromYellow = bChannelAvg - YELLOW_SLEEVE_SIDE;

            if (distanceFromGreen < distanceFromRed && distanceFromGreen < distanceFromYellow) {
                color = SleeveColor.GREEN;
            } else if (distanceFromRed < distanceFromGreen && distanceFromRed < distanceFromYellow) {
                color = SleeveColor.RED;
            } else if (distanceFromYellow > distanceFromRed) {
                color = SleeveColor.YELLOW;
            }

            telemetry.addData("Color detected", color);
            telemetry.update();


            return input;
        }
    }
}