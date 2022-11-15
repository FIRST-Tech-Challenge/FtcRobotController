//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous
//public class openCVTest extends OpMode {
//
//    OpenCvWebcam webcam1 = null;
//
//    @Override
//    public void init() {
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        webcam1.setPipeline(new pipeline());
//
//        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
//            }
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//    }
//
//    @Override
//    public void loop() {
//
//    }
//
//    class pipeline extends OpenCvPipeline {
//        private Mat YCbCr = new Mat();
//        private Mat leftCrop;
//        private Mat middleCrop;
//        private Mat rightCrop;
//        private double leftavgfin;
//        private double middleavgfin;
//        private double rightavgfin;
//        private Mat output = new Mat();
//        private Scalar rectColor = new Scalar(255.0, 174.0, 0.0);
//
//        public Mat processFrame(Mat input) {
//            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//            telemetry.addLine("Pipeline Running");
//
//            Rect leftRect = new Rect(1, 1, 212, 359);
//            Rect middleRect = new Rect(213,1,212,359);
//            Rect rightRect = new Rect(426, 1, 212, 359);
//
//            input.copyTo(output);
//            Imgproc.rectangle(output, leftRect, rectColor, 2);
//            Imgproc.rectangle(output, middleRect, rectColor, 2);
//            Imgproc.rectangle(output, rightRect, rectColor, 2);
//
//            leftCrop = YCbCr.submat(leftRect);
//            middleCrop = YCbCr.submat(middleRect);
//            rightCrop = YCbCr.submat(rightRect);
//
//            Core.extractChannel(leftCrop, leftCrop, 2);
//            Core.extractChannel(middleCrop, middleCrop, 2);
//            Core.extractChannel(rightCrop, rightCrop, 2);
//
//            Scalar leftavg = Core.mean(leftCrop);
//            Scalar middleavg = Core.mean(middleCrop);
//            Scalar rightavg = Core.mean(rightCrop);
//
//            leftavgfin = leftavg.val[0];
//            middleavgfin = middleavg.val[0];
//            rightavgfin = rightavg.val[0];
//
//            if (leftavgfin < rightavgfin && leftavgfin < middleavgfin) {
//                telemetry.addLine("Left");
//            } else if (rightavgfin < leftavgfin && rightavgfin < middleavgfin) {
//                telemetry.addLine("Right");
//            } else {
//                telemetry.addLine("Middle");
//            }
//            return(output);
//        }
//    }
//}
