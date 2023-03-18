package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "PoleAlignTest")
//@Disabled
public class PoleAlignmentTest extends LinearOpMode {
    private PoleDetectionPipeline opencv = null;
    private double centerPosX = 0;
    OpenCvWebcam webcam = null;

    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        opencv = new PoleDetectionPipeline();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
                webcam.setPipeline(opencv);
                //start streaming the camera
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                //if you are using dashboard, update dashboard camera view
                /*FtcDashboard.getInstance().startCameraStream(webcam, 5);*/
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        drive = new SampleMecanumDrive(hardwareMap);


//        initialize camera and pipeline
        waitForStart();
        while (opModeIsActive()) {
            double PIXELS_PER_DEGREE = 26.70190509190664625235159202721;
            double poleX = opencv.poleCenterX;
            double poleWidth = opencv.poleWidth;
            double errorX = poleX - 640;
            double POLE_DISTANCE_CONSTANT = 0.02;

            if (centerPosX > 750 || centerPosX < 550) {
                centerPosX = opencv.poleCenterX;
                telemetry.addData("PoleCenterX", centerPosX);
                telemetry.addLine("Not Equal");

                drive.turn(Math.toRadians((centerPosX - 640) / PIXELS_PER_DEGREE));

                telemetry.update();
            } else {
                centerPosX = opencv.poleCenterX;
                telemetry.addData("PoleCenterX", centerPosX);
                telemetry.addLine("EQUAL");

                telemetry.update();

                drive.leftFront.setPower(0);
                drive.rightFront.setPower(0);
                drive.leftRear.setPower(0);
                drive.rightRear.setPower(0);
            }
        }
//        stopStreaming
        webcam.stopStreaming();
    }
}
