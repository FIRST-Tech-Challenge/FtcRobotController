package org.firstinspires.ftc.teamcode.opmode;

import static org.opencv.core.Core.inRange;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="cameraTest", group="Linear Opmode")
public class camera_pixel_detection extends OpMode {
    boolean lastA = false, lastB = false, lastY = false, lastX = false ,
            lastLeftTrigger = false, lastRightTrigger = false, lastLeftBumper = false, lastRightBumper = false;
    int cameraMonitorViewId;
    int l_or_u = 1;
    public static int choose_H_S_V = 0; // first arm weight KG
    WebcamName webcamName;
    OpenCvCamera camera;
    double[] lowerPvals = new double[]{0, 0, 0};
    double[] upperPvals = new double[]{60, 255, 255};
    Scalar lowerP = new Scalar(250, 50, 76);
    Scalar upperP = new Scalar(300, 180, 180);

//    Scalar lowerW = new Scalar(0, 0, 200); the real lowerW
//    Scalar upperw = new Scalar(360, 25, 240); the real upperW
//    double[] lowerWvals = new double[]{0, 0, 200}; the real upperWvals
//    double[] upperWvals = new double[]{360, 25, 240}; the real upperWvals

//    double[] lowerGvals = new double[]{0, 0, 200}; the real lowerGvals
//    double[] upperGvals = new double[]{360, 25, 240}; the real upperGvals
//    Scalar lowerG = new Scalar(50, 75, 80); the real lowerG
//    Scalar upperG = new Scalar(150, 250, 250); the real upperG



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
        telemetry.addData("lowerP: ", lowerP);
        telemetry.addData("upperP: ", upperP);
        telemetry.addData("HSV number ", choose_H_S_V);
        telemetry.addData("lower or upper ", l_or_u);
        telemetry.addLine("a=+1 \n b=-1 \n x=+10 \n y=-10 \n left_trigger > 0.1=Choose H or S or V \n lower = 1   upper = 2");
        if(gamepad1.left_bumper && !lastLeftBumper && l_or_u > 1) {
            l_or_u -= 1;
        }lastLeftBumper = gamepad1.left_bumper;

        if(gamepad1.right_bumper && !lastRightBumper && l_or_u < 2){
            l_or_u += 1;
        }lastRightBumper = gamepad1.right_bumper;

        if(gamepad1.left_trigger > 0.1 && !lastLeftTrigger && choose_H_S_V > 0){
            choose_H_S_V -= 1;
        }lastLeftTrigger =gamepad1.left_trigger > 0.1;

        if(gamepad1.right_trigger > 0.1 && !lastRightTrigger){
            choose_H_S_V += 1;
        }lastRightTrigger=gamepad1.right_trigger > 0.1 && choose_H_S_V < 2;



        if(gamepad1.a && !lastA){
            if(l_or_u == 2){
                upperPvals[choose_H_S_V]++;}
            if(l_or_u == 1){
                lowerPvals[choose_H_S_V]++;}
        }lastA=gamepad1.a;

        if(gamepad1.b && !lastB){
            if(l_or_u == 2){
            upperPvals[choose_H_S_V]--;}
            if(l_or_u == 1){
            lowerPvals[choose_H_S_V]--;}
        }lastB=gamepad1.b;

        if(gamepad1.x && !lastX){
            if(l_or_u == 2){
            upperPvals[choose_H_S_V] += 10;}
            if(l_or_u == 1){
            lowerPvals[choose_H_S_V] += 10;}
        }lastX=gamepad1.x;

        if(gamepad1.y && !lastY){
            if(l_or_u == 2){
            upperPvals[choose_H_S_V] -= 10;}
            if(l_or_u == 1){
            upperPvals[choose_H_S_V] -= 10;}
        }lastY=gamepad1.y;

        lowerP.set(lowerPvals);
        upperP.set(upperPvals);
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
        public Mat processframeG(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            //                Mat kernel = Imgproc.getStructuringElement(0, new Size(5.0,5.0),new Point(5, 5));
            inRange(hsv, lowerP, upperP, binaryG);
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