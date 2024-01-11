package org.firstinspires.ftc.teamcode.opmode;

import static org.opencv.core.Core.inRange;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Scalar;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="cameraTest", group="Linear Opmode")
public class camera_pixel_detection extends OpMode {
    boolean lastA = false, lastB = false, lastY = false, lastX = false ,lastleft_trigger = false, lastright_trigger, lastleft_bumper = false, lastright_bumper = false;
    int cameraMonitorViewId;
    public static int choose_H_S_V = 0;

    public static int choose_upper_or_lower = 0;
    WebcamName webcamName;
    OpenCvCamera camera;
    double[] lowerPvals = new double[]{50, 20, 20};
    double[] upperPvals = new double[]{250, 125, 200};
    Scalar lowerP = new Scalar(50, 20, 20);
    Scalar upperP = new Scalar(250, 125, 200);
//    Scalar upperG = new Scalar(150, 250, 250); the real upperG
//    Scalar lowerG = new Scalar(50, 75, 80); the real lowerG
//    Scalar lowerW = new Scalar(0, 0, 200); todo:not best numbers, check better
//    Scalar upperW = new Scalar(360, 25, 240);todo:not best numbers, check better
//    double[] lowerPvals = new double[]{0, 0, 200};
//    double[] upperWvals = new double[]{360, 25, 240};


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
        telemetry.addData("H_S_V_number ", choose_H_S_V);
        telemetry.addData("lower = 0, upper = 1 ", choose_upper_or_lower);
        telemetry.addLine("a = +1 \n b = -1 \n x = +10 \n y = -10 \n" +
                " triggers=Choose H or S or V (H=0 S=1 V=2) \n bumpers = upper/lower ") ;
        if(gamepad1.left_bumper && !lastleft_bumper && choose_upper_or_lower > 0){
            choose_upper_or_lower -= 1;
        }lastleft_bumper=gamepad1.left_bumper;

        if(gamepad1.right_bumper && !lastright_bumper && choose_upper_or_lower < 1){
            choose_upper_or_lower += 1;
        }lastright_bumper=gamepad1.right_bumper;

        if(gamepad1.left_trigger > 0.1 && !lastleft_trigger && choose_H_S_V > 0){
            choose_H_S_V -= 1;
        }lastleft_trigger=gamepad1.left_trigger > 0.1;

        if(gamepad1.right_trigger>0.1 && !lastright_trigger && choose_H_S_V < 2){
            choose_H_S_V += 1;
        }lastright_trigger=gamepad1.right_trigger > 0.1;


        if(gamepad1.a && !lastA){
            if(choose_upper_or_lower == 1){
                upperPvals[choose_H_S_V]++;}

            if(choose_upper_or_lower == 0){
                lowerPvals[choose_H_S_V]++;}
        }
        lastA=gamepad1.a;

        if(gamepad1.b && !lastB){
            if(choose_upper_or_lower == 1){
                upperPvals[choose_H_S_V]--;}

            if(choose_upper_or_lower == 0){
                lowerPvals[choose_H_S_V]--;}
        }lastB=gamepad1.b;

        if(gamepad1.x && !lastX){
            if(choose_upper_or_lower == 1){
                upperPvals[choose_H_S_V] += 10;}

            if(choose_upper_or_lower == 0){
                lowerPvals[choose_H_S_V] += 10;}
        }lastX=gamepad1.x;

        if(gamepad1.y && !lastY){
            if(choose_upper_or_lower == 1){
                upperPvals[choose_H_S_V] -= 10;}

            if(choose_upper_or_lower == 0){
                lowerPvals[choose_H_S_V] -= 10;}
        }lastY=gamepad1.y;
        upperP.set(upperPvals);
        lowerP.set(lowerPvals);
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