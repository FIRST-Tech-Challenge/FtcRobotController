package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
@TeleOp(name = "Goal Detection")
public class GoalDetectionTeleOp extends MasterTeleOp {

    GoalDetectionOpenCV goalDetector = new GoalDetectionOpenCV();
    OpenCvCamera webcam;
    double[] values = new double[3];
    @Override
    public void runOpMode() throws InterruptedException {

        initializeHardware();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Top Webcam"), cameraMonitorViewId);
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
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        webcam.setPipeline(goalDetector);


        telemetry.addLine("Waiting for start");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Started");
            //telemetry.addData("Blue area", goalDetector.blueArea);
            telemetry.addData("Number of contours", goalDetector.whiteContours.size());
            telemetry.addData("Max Area", goalDetector.whiteRect.area());


            int x1 = goalDetector.whiteGoal.x + goalDetector.blueRect.x;
            // blue width - white width - white x
            int x2 = (goalDetector.blueRect.width - goalDetector.whiteGoal.width - goalDetector.whiteGoal.x)
                    + (goalDetector.displayMat.width() - (goalDetector.blueRect.x + goalDetector.blueRect.width));

            int xError = (x1 - x2) - 118;
            int yError = 65 - goalDetector.whiteGoal.height;
            double error = Math.hypot(xError, yError);
            double drivePower = xError * 0.0015;
            double angle = Math.atan2(yError, xError);
            /*double xDrivingPower = xError * 0.001;



            double yDrivingPower = yError * 0.001;
            double angle = Math.atan2(yDrivingPower, xDrivingPower);
            double drivePower = Math.hypot(xDrivingPower, yDrivingPower);*/



            if (gamepad1.dpad_up) {
                goalDetector.sLower ++;
                sleep(300);
            } else if (gamepad1.dpad_down) {
                goalDetector.sLower --;
                sleep(300);
            } else if (gamepad1.dpad_right) {
                goalDetector.sUpper ++;
                sleep(300);
            } else if (gamepad1.dpad_left) {
                goalDetector.sUpper --;
                sleep(300);
            } else if (gamepad1.y) {
                goalDetector.vLower ++;
                sleep(300);
            } else if (gamepad1.a) {
                goalDetector.vLower --;
                sleep(300);
            } else if (gamepad1.x) {
                goalDetector.vUpper --;
                sleep(300);
            } else if (gamepad1.b) {
                goalDetector.vUpper ++;
                sleep(300);
            }

            telemetry.addData("Driving Power", xError);
            telemetry.addData("X1", x1);
            telemetry.addData("X2", x2);
            telemetry.addData("White height", goalDetector.whiteGoal.height);
            telemetry.addData("White width", goalDetector.whiteGoal.width);
            //telemetry.addData("White is in middle range", whiteIsInMiddleRange);
            telemetry.addLine();


            if ((Math.abs(xError) > 10 /*|| Math.abs(yError) > 10*/) && gamepad1.left_trigger != 0) {
                mecanumDrive(0, drivePower, 0);
            } else {
                driveRobot();
            }

            telemetry.update();
            idle();
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();

    }
}
