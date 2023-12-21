package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static org.opencv.core.Core.inRange;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Scalar;

import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="cameraTest", group="Linear Opmode")
public class camera_pixel_detection extends OpMode {
    int cameraMonitorViewId;
    WebcamName webcamName;
    OpenCvCamera camera;
    boolean lastA = false, lastB = false, lastY = false, lastX = false;
    double[] lowerPvals = new double[]{0, 0, 0};
    double[] upperPvals = new double[]{60, 255, 255};
    Scalar lowerP = new Scalar(290, 40, 50); //48 65 60
    Scalar upperP = new Scalar(320, 100, 65); //65 100 100
    Scalar lowerG = new Scalar(32,104,18);

    Scalar upperG = new Scalar(123,171,111);
    Scalar lowerO = new Scalar(15, 75, 45);
    Scalar upperO = new Scalar(35, 100, 56);


    @Override
    public void init() {
        /**
         To stream to the driver station, simply make sure that you've started a streaming session during the OpMode initialization.
         When running your OpMode, do NOT press start. Only press INIT.
         While the OpMode is in the INIT phase, you can open the overflow menu on the DriverStation and select the "Camera Stream" option.
         This provides a tap-to-refresh view of the pipeline output. Select Camera Stream again to close the preview and continue the OpMode.
         */
        webcamName = hardwareMap.get(WebcamName.class, "camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        telemetry.clearAll();
        telemetry.update();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                try {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

                } catch (Exception e) {
                    telemetry.addData("error:", e.getMessage());
                    telemetry.update();
                }
                // Usually this is where you'll want to start streaming from the camera (see section 4 in docs)

                telemetry.update();

                camera.setPipeline(new Pipeline1());
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("camera", "failed to open");
                telemetry.update();
            }
        });
    }

    @Override
    public void init_loop() {
        if (gamepad1.a && !lastA) {
            lowerPvals[0]++;
        }
        lastA = gamepad1.a;
        if (gamepad1.b && !lastB) {
            lowerPvals[0]--;
        }
        lastB = gamepad1.b;
        lowerP.set(lowerPvals);
        if (gamepad1.x && !lastX) {
            upperPvals[0]++;
        }
        lastX = gamepad1.x;
        if (gamepad1.y && !lastY) {
            upperPvals[0]--;
        }
        lastY = gamepad1.y;
        upperP.set(upperPvals);
        telemetry.addData("upperY: ", upperP.val[0]);
        telemetry.addData("lowerY: ", lowerP.val[0]);
    }

    @Override
    public void loop() {

    }

    public class Pipeline1 extends OpenCvPipeline {

        public Mat binaryP = new Mat(640, 480, CvType.CV_8UC1);
        public Mat binaryG = new Mat(640, 480, CvType.CV_8UC1);
        public Mat binaryO = new Mat(640, 480, CvType.CV_8UC1);
        Mat hsv = new Mat(640, 480, CvType.CV_8UC1);
        Mat mat = new Mat(640, 480, CvType.CV_8UC1);
        Mat binaryAfterP = new Mat(640, 480, CvType.CV_8UC1);
        Mat binaryAfterG = new Mat(640, 480, CvType.CV_8UC1);
        Mat binaryAfterO = new Mat(640, 480, CvType.CV_8UC1);

        List contours;

        public Mat processFrameP(Mat input) {
            try {

                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//                Mat kernel = Imgproc.getStructuringElement(0, new Size(5.0,5.0),new Point(5, 5));
                inRange(hsv, lowerP, upperP, binaryP);
//                Imgproc.erode(binaryP,binaryAfterP,kernel);
            } catch (Exception e) {
                telemetry.addData("process frame error:", e.getMessage());
                telemetry.update();
            }

            return binaryP;
        }

        public Mat processframeO(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            //                Mat kernel = Imgproc.getStructuringElement(0, new Size(5.0,5.0),new Point(5, 5));
            inRange(hsv, lowerO, upperO, binaryO);
//                            Imgproc.erode(binaryO,binaryAfterO,kernel);
            return binaryO;
        }

        public Mat processframeG(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            //                Mat kernel = Imgproc.getStructuringElement(0, new Size(5.0,5.0),new Point(5, 5));
            inRange(hsv, lowerG, upperG, binaryG);
            //                Imgproc.erode(binaryG,binaryAfterG,kernel);
            return binaryG;
        }

        @Override
        public Mat processFrame(Mat input) {
            processframeG(input);
//            processFrameP(input);
//            processframeO(input);
            for (Mat mat1 : Arrays.asList(binaryG, binaryO, binaryP)) {
                return mat1;
            }
            return input;
        }
    }
}