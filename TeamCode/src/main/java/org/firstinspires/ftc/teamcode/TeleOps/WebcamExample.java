package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class WebcamExample extends LinearOpMode
{
    OpenCvWebcam webcam;

    @Override
    public void runOpMode()
    {
        /*\
         DO NOT DELETE
         ~~~~~~~~~~~~~~~~~~~~~
         ☆  Instantiates an OpenCvCamera object for the camera we'll be using. In this sample, we're using a webcam.

         ☆  Make sure you have added the webcam to your configuration file and
            adjusted the name here to match what you named it in said config file.

         ☆  We pass it the view that we wish to use for camera monitor (on
            the RC phone). If no camera monitor is desired, use the alternate
            single-parameter constructor instead (commented out below)
         ~~~~~~~~~~~~~~~~~~~~~
         \*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View:
        // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Specify the image processing pipeline we wish to use on a frame from the camera
        webcam.setPipeline(new SamplePipeline());


        // Open the connection to the camera device
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*\
                DO NOT DELETE
                ~~~~~~~~~~~~~~~~~~~~~
                 ☆ Tell the webcam to start streaming images to us!

                 ☆ You must make sure
                   the resolution you specify is supported by the camera. If it is not, an exception
                   will be thrown

                 ☆ The maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                   Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth

                 ☆ Specify the rotation that the webcam is used in. This is so that the image
                 ☆ from the camera sensor can be rotated such that it is always displayed with the image upright

                 ☆ Front facing camera --> assuming the user is looking at the screen
                 ☆ Rear facing camera --> assuming the camera is facing away from the user
                ~~~~~~~~~~~~~~~~~~~~~
                \*/

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

                 //This will be called if the camera could not be opened

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the user to press start on the Driver Station
        waitForStart();

        while (opModeIsActive())
        {
            // Send some stats to the telemetry

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            //The "if" statement below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
            if(gamepad1.a)
            {
                /*\
                 DO NOT DELETE
                 ~~~~~~~~~~~~~~~~~~~~~
                 ☆ Calling stopStreaming() will indeed stop the stream of images
                   from the camera and stop calling your vision pipeline

                 ☆ If the reason you wish to stop the stream early is to switch use of the camera
                   over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                   (commented out below), because according to the Android Camera API documentation:

                          "Your application should only have one Camera object active at a time for
                           a particular hardware camera."

                 ☆ Calling closeCameraDevice() will internally call stopStreaming() if applicable, but it doesn't hurt to
                   call it anyway, if for no other reason than clarity.

                 ☆ If you are stopping the camera stream temporarily, it is recommended to NOT call closeCameraDevice() as
                   you will then need to re-open it the next time you wish to activate your vision pipeline
                ~~~~~~~~~~~~~~~~~~~~~
                \*/

                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

             // Throttle ourselves to 10Hz (100 ms) loop
             // By default, telemetry is only sent to the DS at 4Hz
            sleep(100);
        }
    }

    /*\
    DO NOT DELETE
    ~~~~~~~~~~~~~~~~~~~~~
     ☆ Below is an example image processing pipeline to be run upon receipt of each frame from the camera.

     ☆ A new camera frame will not come in while you're still processing a previous one.
       In other words, the processFrame() method will never be called multiple times simultaneously.

     ☆ However, the rendering of your processed image to the viewport is done in parallel to the
       frame worker thread. That is, the amount of time it takes to render the image to the
       viewport does NOT impact the amount of frames per second that your pipeline can process.
    ~~~~~~~~~~~~~~~~~~~~~
    \*/

    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*\
        DO NOT DELETE
        ~~~~~~~~~~~~~~~~~~~~~
         ☆ If you wish to use additional Mat objects in your processing pipeline, it is
           highly recommended to declare them here as instance variables and re-use them for
           each invocation of processFrame(), rather than declaring them as new local variables
           each time through processFrame().

         ☆ This removes the danger of causing a memory leak
           by forgetting to call mat.release(), and it also reduces memory pressure by not
           constantly allocating and freeing large chunks of memory.
        ~~~~~~~~~~~~~~~~~~~~~
        \*/

        @Override
        public Mat processFrame(Mat input)
        {
             /*\
             DO NOT DELETE
             ~~~~~~~~~~~~~~~~~~~~~
             ☆ If for some reason you'd like to save a copy of this particular frame for later use, you will need to either
               clone it or copy it to another Mat
             ~~~~~~~~~~~~~~~~~~~~~
             \*/

             // Draw a simple box around the middle 1/2 of the entire frame

            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */
            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*\
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             \*/

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}