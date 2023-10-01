package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TestOpenCV.CircleDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutonomousOpenCV extends autonomous {
    private OpenCvWebcam webcam;
    private CircleDetection circleDetection;
    private DrivingFunctions df;

    private ElapsedTime runtime = new ElapsedTime();

    private void Initialize()
    {
        df = new DrivingFunctions(this);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new CircleDetection(telemetry);
        webcam.setPipeline(circleDetection);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
    @Override
    public void runOpMode() {
        Initialize();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        int tries = 0;
        // Waits until the camera detects the ball, up to 5 seconds
        while(circleDetection.ballPosition == BallPosition.UNDEFINED && tries < 50) {
            sleep(100);
            tries++;
            UpdateTelemetry();
        }

        if(circleDetection.ballPosition == BallPosition.UNDEFINED)
            circleDetection.ballPosition = BallPosition.LEFT; // Ball not found, makes a guess to the left

        df.driveForward(0.5,1000);

        if(circleDetection.ballPosition == BallPosition.LEFT) {
            df.rotateLeft(0.5, 300);
        }
        if(circleDetection.ballPosition == BallPosition.RIGHT) {
            df.rotateRight(0.5, 300);
        }
        df.driveForward(0.5,1000);
        UpdateTelemetry();
        StopStreaming();
    }

    private void StopStreaming()
    {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private void UpdateTelemetry()
    {
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("Circles detected: ", "%d", circleDetection.numCirclesFound);
        telemetry.addData("Circle center = ", "%4.0f, %4.0f", circleDetection.circleCenter.x, circleDetection.circleCenter.y);
        telemetry.addData("Ball Position: ", "%s", circleDetection.ballPosition);
        telemetry.update();
    }
    enum BallPosition {LEFT, CENTER, RIGHT, UNDEFINED};
    public class CircleDetection extends OpenCvPipeline {
        public BallPosition ballPosition = BallPosition.UNDEFINED;
        Mat grayMat = new Mat();
        Mat hsvMaskedMat = new Mat();
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask = new Mat();
        Mat hsvMat = new Mat();

        public int numCirclesFound = 0;
        public Point circleCenter = new Point(0.0, 0.0);

        private Telemetry telemetry;

        public CircleDetection(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsvMat, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1);
            Core.inRange(hsvMat, new Scalar(160, 70, 50), new Scalar(180, 255, 255), mask2);

            Core.bitwise_or(mask1, mask2, mask);
            hsvMaskedMat.release();
            Core.bitwise_and(input, input, hsvMaskedMat, mask);

            Imgproc.cvtColor(hsvMaskedMat, grayMat, Imgproc.COLOR_RGB2GRAY);

            Imgproc.GaussianBlur(grayMat, grayMat, new org.opencv.core.Size(15.0, 15.0), 2, 2);
            Mat circles = new Mat();
            Imgproc.HoughCircles(grayMat, circles, Imgproc.HOUGH_GRADIENT, 1, 150, 130, 30);

            numCirclesFound = circles.cols();

            for (int i = 0; i < numCirclesFound; i++) {
                double[] data = circles.get(0, i);
                circleCenter = new Point(Math.round(data[0]), Math.round(data[1]));
                ballPosition = circleCenter.x < 300 ? BallPosition.LEFT : (circleCenter.x > 700 ? BallPosition.RIGHT : BallPosition.CENTER);
            }
            return input;
        }
    }
}