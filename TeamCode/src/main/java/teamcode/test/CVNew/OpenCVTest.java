package teamcode.test.CVNew;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCVTest extends LinearOpMode {
    private static final String VISION_TARGET_1_PATH = "";
    OpenCvWebcam webcam;
    Mat visionTarget1 = Imgcodecs.imread(VISION_TARGET_1_PATH);

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        skystoneDetector detector = new skystoneDetector();
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240); //specify cam orientation and calibrate the resolution
        });
        waitForStart();
    }


    enum SkystonePos{
        LEFT, MIDDLE, RIGHT
    }

    class skystoneDetector extends OpenCvPipeline{
        private Mat workingMat = new Mat();
        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(workingMat);
            if(workingMat.empty()){
                return input;
            }
            Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb); //grayscale the image
            Mat left = workingMat.submat(120, 150, 10, 50);
            Mat center = workingMat.submat(120, 150, 80, 120);
            Mat right = workingMat.submat(120, 150, 150, 190);

            Imgproc.rectangle(workingMat, new Rect(10, 120, 40, 30), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMat, new Rect(80, 120, 40, 30), new Scalar(0, 255, 0));
            Imgproc.rectangle(workingMat, new Rect(150, 120, 40, 30), new Scalar(0, 255, 0));
            double leftSum = Core.sumElems(left).val[2];
            double centerSum = Core.sumElems(center).val[2];
            double rightSum = Core.sumElems(right).val[2];

            if(leftSum > rightSum && leftSum > centerSum){

            }




            return workingMat;
        }
    }
}
