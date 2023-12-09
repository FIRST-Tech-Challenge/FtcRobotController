package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class TwoCameraTestAuton extends LinearOpMode {

        OpenCvCamera phoneCam;
        OpenCvCamera webcam;

        @Override
        public void runOpMode()
        {
            /**
             * NOTE: Many comments have been omitted from this sample for the
             * sake of conciseness. If you're just starting out with EasyOpenCV,
             * you should take a look at {@link InternalCamera1Example} or its
             * webcam counterpart, {@link WebcamExample} first.
             */

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            /**
             * This is the only thing you need to do differently when using multiple cameras.
             * Instead of obtaining the camera monitor view and directly passing that to the
             * camera constructor, we invoke {@link OpenCvCameraFactory#splitLayoutForMultipleViewports(int, int, OpenCvCameraFactory.ViewportSplitMethod)}
             * on that view in order to split that view into multiple equal-sized child views,
             * and then pass those child views to the constructor.
             */
            int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                    .splitLayoutForMultipleViewports(
                            cameraMonitorViewId, //The container we're splitting
                            2, //The number of sub-containers to create
                            OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally

            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), viewportContainerIds[0]);
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), viewportContainerIds[1]);

            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    phoneCam.setPipeline(new UselessGreenBoxDrawingPipeline());
                    phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.setPipeline(new UselessGreenBoxDrawingPipeline());
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

            waitForStart();

            while (opModeIsActive())
            {
                telemetry.addData("Internal cam FPS", phoneCam.getFps());
                telemetry.addData("Webcam FPS", webcam.getFps());
                telemetry.update();

                sleep(100);
            }
        }

        class UselessGreenBoxDrawingPipeline extends OpenCvPipeline
        {
            @Override
            public Mat processFrame(Mat input)
            {
                Imgproc.rectangle(
                        input,
                        new Point(
                                input.cols()/4,
                                input.rows()/4),
                        new Point(
                                input.cols()*(3f/4f),
                                input.rows()*(3f/4f)),
                        new Scalar(0, 255, 0), 4);

                return input;
            }
        }
    }





