package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Continuously adaptive meanshift")
public class OpenCVCamshiftMaybe extends LinearOpMode {
    public OpenCvWebcam webcam;

    public CamshiftPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline = new CamshiftPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
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

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis",3);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
    public static class CamshiftPipeline extends OpenCvPipeline {
        Telemetry telemetry;
        public CamshiftPipeline(Telemetry telemetry) {this.telemetry = telemetry;}

        public enum Position {

        }
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);

        @Override
        public Mat processFrame(Mat input) {

            Mat hsv_roi = new Mat(), mask = new Mat(), roi;
            // take the first frame of the video
            //setup initial location of window
            Rect track_window = new Rect(300, 200, 100, 50);
            // set up the ROI for tracking
            roi = new Mat(input, track_window);
            Imgproc.cvtColor(roi, hsv_roi, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsv_roi, new Scalar(0, 60, 32), new Scalar(180, 255, 255), mask);
            MatOfFloat range = new MatOfFloat(0, 256);
            Mat roi_hist = new Mat();
            MatOfInt histSize = new MatOfInt(180);
            MatOfInt channels = new MatOfInt(0);
            Imgproc.calcHist(Arrays.asList(hsv_roi), channels, mask, roi_hist, histSize, range);
            Core.normalize(roi_hist, roi_hist, 0, 255, Core.NORM_MINMAX);
            // Setup the termination criteria, either 10 iteration or move by atleast 1 pt
            TermCriteria term_crit = new TermCriteria(TermCriteria.EPS | TermCriteria.COUNT, 10, 1);
            while (true) {
                Mat hsv = new Mat(), dst = new Mat();
                if (input.empty()) {
                    break;
                }
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
                Imgproc.calcBackProject(Arrays.asList(hsv), channels, roi_hist, dst, range, 1);
                // apply camshift to get the new location
                RotatedRect rot_rect = Video.CamShift(dst, track_window, term_crit);
                // Draw it on image
                Point[] points = new Point[4];
                rot_rect.points(points);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, points[i], points[(i + 1) % 4], new Scalar(255, 0, 0), 2);
                }
            }
            System.exit(0);
            return null;
        }
    }
}