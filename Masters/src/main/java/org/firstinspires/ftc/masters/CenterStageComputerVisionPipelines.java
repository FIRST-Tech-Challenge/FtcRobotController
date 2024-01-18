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

import com.acmerobotics.dashboard.FtcDashboard;
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


public class CenterStageComputerVisionPipelines {

    public enum PipelineType {PROP_FIND}

    //    Declare webcam
    public OpenCvWebcam webcam;
//    public OpenCvWebcam backWebcam;

    //    Initial declaration of pipelines (One for each we use)
    public FrontPipeline backPipeline;
    public PropFind propFind;

    Telemetry telemetry;

    boolean error = false;
    public PipelineType sleeveWebcamPipeline = PipelineType.PROP_FIND;

//    Random useful tidbit of information, to access the camera stream, hit innit, then
//    press the ellipsis button on the top right of the screen, then select "Camera Stream".
//    Follow same path to turn it off


    public CenterStageComputerVisionPipelines(HardwareMap hardwareMap, Telemetry telemetry) {

//        Get and store camera monitor view id.
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontWebcam"));
        this.telemetry = telemetry;

        propFind = new PropFind(telemetry);
        propFind.setPipelineType(PipelineType.PROP_FIND);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        backWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "backWebcam"), cameraMonitorViewId);

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
                webcam.setPipeline(propFind);
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

//        backWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                telemetry.addData("sleeve webcam open", "yes");
//                backWebcam.setPipeline(backPipeline);
//                backWebcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addLine("Can't open sleeve camera");
//                telemetry.update();
//            }
//        });


    }

    public boolean isError() {
        return error;
    }

    public void setPropFindPipeline() {
        sleeveWebcamPipeline = PipelineType.PROP_FIND;
        propFind.setPipelineType(sleeveWebcamPipeline);

    }

    public void stopCamera() {
        webcam.stopStreaming();
        webcam.setPipeline(new DoNothingPipeline());
    }
//
//    public void stopBackCamera() {
//        backWebcam.stopStreaming();
//        backWebcam.setPipeline(new DoNothingPipeline());
//    }

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
    public enum pos {
        LEFT,
        MID,
        RIGHT,
    }
    public class PropFind extends OpenCvPipeline {

        private final Rect interestMid = new Rect(178, 180, 32, 50);
        private final Rect interestRight = new Rect(460, 230, 32, 50);
        private final Rect pix = new Rect(1,1,1,1);



        public pos position = pos.LEFT;

        Telemetry telemetry;
        PipelineType pipelineType = PipelineType.PROP_FIND;


        public PropFind(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        public void setPipelineType(PipelineType pipelineType) {
            this.pipelineType = pipelineType;
        }

        Mat LAB = new Mat(), dst = new Mat();
        Mat A = new Mat();
        Mat B = new Mat();
        Mat HSV = new Mat();
        Mat H = new Mat();
        Mat S = new Mat();
        Mat V = new Mat();



        Mat region_a_mid = new Mat();
        Mat region_b_mid = new Mat();
        Mat region_a_right = new Mat();
        Mat region_b_right = new Mat();

        int avg_a_mid = 0;
        int avg_b_mid = 0;
        int avg_a_right = 0;
        int avg_b_right = 0;

        Mat mask = new Mat(), diff_im = new Mat();

        int detected;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        void inputToLAB(Mat input) {

            Imgproc.cvtColor(input, LAB, Imgproc.COLOR_RGB2Lab);
            Core.extractChannel(LAB, A, 0);
            Core.extractChannel(LAB, B, 1);
        }

        void inputToHSV(Mat input) {
            Imgproc.cvtColor(input,HSV,Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(HSV, H, 0);
            Core.extractChannel(HSV, S, 1);
            Core.extractChannel(HSV, V, 2);
        }


        @Override
        public Mat processFrame(Mat input) {

            inputToLAB(input);
            inputToHSV(input);

            // Useful for red isolation
            //Core.inRange(HSV, new Scalar(150,50,0), new Scalar(180,255,255), mask); //Black out Red

            // Blue prop
            Core.inRange(HSV, new Scalar(105,50,0), new Scalar(115,255,255), mask); //Black out Red

/*
        Core.inRange(HSV, new Scalar(47,50,0), new Scalar(80,255,255), mask); //Black out green pixel
        Core.inRange(HSV, new Scalar(15,50,0), new Scalar(35,255,255), mask); //Black out yellow pixel
        Core.inRange(HSV, new Scalar(110,50,0), new Scalar(150,255,255), mask); //Black out purple pixel
        Core.inRange(HSV, new Scalar(0,0,100), new Scalar(120,30,255), mask); //Black out white pixel
*/

            diff_im = new Mat();
            Core.add(diff_im, Scalar.all(0), diff_im);
            Core.bitwise_not(mask,mask);
            input.copyTo(diff_im, mask);
            diff_im.copyTo(input);
            diff_im.release();

            inputToHSV(input);

            region_a_mid = H.submat(interestMid);
            region_b_mid = S.submat(interestMid);
            region_a_right = H.submat(interestRight);
            region_b_right = S.submat(interestRight);

            avg_a_mid = (int) Core.mean(region_a_mid).val[0];
            avg_b_mid = (int) Core.mean(region_b_mid).val[0];
            avg_a_right = (int) Core.mean(region_a_right).val[0];
            avg_b_right = (int) Core.mean(region_b_right).val[0];

            if (avg_a_mid<5) {
                position = pos.MID;
            } else if (avg_a_right<5) {
                position = pos.RIGHT;
            } else {
                position = pos.LEFT;
            }

            telemetry.update();

            Imgproc.rectangle(input, new Point(interestMid.x, interestMid.y), new Point(interestMid.x + interestMid.width, interestMid.y+ interestMid.height), new Scalar(0,255,0),1 );
            Imgproc.rectangle(input, new Point(interestRight.x, interestRight.y), new Point(interestRight.x + interestRight.width, interestRight.y+ interestRight.height), new Scalar(0,255,0),1 );


            return input; //dst
        }
    }
}