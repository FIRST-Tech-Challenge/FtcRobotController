//package org.firstinspires.ftc.teamcode.drivecode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.VisionProcessor;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.opencv.core.Mat;
//import org.opencv.android.Utils;
//import org.opencv.core.Core;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
////import org.openftc.easyopencv.OpenCvCameraFactory;
////OpenCvCameraFactory is bugged on OBJ, workaround is to either port EOCV pipeline to Vision processor or use ADS
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous(name = "AutonTFlippyflappityQUESTIONQUESTION", group = "Robot")
//@Disabled
//public class AutonTFlippyflappityQUESTIONQUESTION extends LinearOpMode {
//
//    OpenCvWebcam webcam1 = null;
//
//    @Override
//    public void runOpMode() {
//
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, /* deviceName: */ "Webcam 1");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIndentifier(/* name: */ "cameraMonitorViewId",/* defType:*/ "id", hardwareMap.appContext.getPackageName());
//        //webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//
//        //webcam1.setPipeline(new examplePipeline()); { pipelines not working??
//
//
//        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            public void onOpened() {
//                webcam1.startStreaming(/*X*/ 640, /*Y*/ 360,OpenCvCameraRotation.UPRIGHT);
//            }
//
//            public void onError(int errorCode) {
//            }
//        });
//
//    }
//
////     @Override
////     public void loop() {
//
//
////   }
//
//    class examplePipeline extends OpenCvPipeline{
//        mat YCbCr = new Mat();
//        Mat leftCrop;
//        Mat rightCrop;
//        double leftavgfin;
//        double rightavgfin;
//        Mat outPut = new Mat();
//        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);
//
//
//
//
//
//        public Mat processFrame(Mat input){
//
//            Imgproc.cvtColor(input,YCbCr, Imgproc.COLOR_RGB2YCrCb);
//            telementry.addLine("pipeline running yippee!!");
//
//            Rect leftRect = new Rect(/*x*/ 1, /*y*/1, /*width*/319, /*height*/359);
//            Rect rightRect = new Rect(/*x*/320, /*y*/1, /*width*/319, /*height*/359);
//
//            input.copyTo(outPut);
//            Imgproc.rectangle(outPut, leftRect, rectColor, /*thickness*/2);
//            Imgproc.rectangle(outPut, rightRect, rectColor, /*thickness*/2);
//
//            leftCrop = YCbCr.submat(leftRect);
//            rightCrop = YCbCr.submat(rightRect);
//
//            Core.extractChannel(leftCrop, leftCrop, /*color of interest, 2 is red*/2);
//            Core.extractChannel(rightCrop, rightCrop, 2);
//
//            Scalar leftavg = Core.mean(leftCrop);
//            Scalar rightavg = Core.mean(rightCrop);
//
//            if (leftavgfin > rightavgfin){
//                telemetry.addLine("Object on the left");
//            }
//
//            else{
//                telemetry.addLine("Object on the right");
//            }
//
//            return(outPut);
//    }
//
//    }
//}
//
//
//
//*/
//
//
