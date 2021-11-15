package org.firstinspires.ftc.teamcode.mentor.samples.FaceDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mentor.samples.HardwareMaps.InternalCameraHardwareMap;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.dashboard.FtcDashboard;

@Autonomous(name="Auto: Face Detector", group="Mentor")
public class FaceDetectorAutoMode extends LinearOpMode {
    // Handle hardware stuff...

    int width = 320;
    int height = 240;
    // store as variable here so we can access the location
    FaceDetector detector = new FaceDetector(width);
    //OpenCvCamera phoneCam;


    @Override
    public void runOpMode() {

        InternalCameraHardwareMap robot = new InternalCameraHardwareMap();
        robot.init(hardwareMap);
        robot.phoneCam.setPipeline(detector);

        robot.phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                robot.phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.addLine("Waiting for start");
        telemetry.update();
        dashboard.startCameraStream(robot.phoneCam, 10);
        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", robot.phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", robot.phoneCam.getFps()));
            telemetry.addData("Total frame time ms", robot.phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", robot.phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", robot.phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", robot.phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();


            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }


    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }
    }
}
