/// REGULA NR 1 - TOMA SCRIE COD PERFECT
package org.firstinspires.ftc.teamcode2024;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.kotlin.extensions.geometry.Pose2dExtKt;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode2024.drive.DriveConstants;
import org.firstinspires.ftc.teamcode2024.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode2024.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode2024.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Autonomie bazata albastru lung", group="Linear Opmode")
public class BlueLung extends GlobalScope2024 {
    int pozitie = 0;
    int cx = 0, cy = 0;
    double maxArea = 0;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {

        class SamplePipeline extends OpenCvPipeline {
            boolean viewportPaused;

            boolean zoneFound = false;


            @Override
            public Mat processFrame(Mat input) {

                if (true) {
                    Mat hsvImage = new Mat();
                    Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2BGR);

                    // Define lower and upper bounds for red color in HSV
                    Scalar lowerRed = new Scalar(150, 50, 50);
                    Scalar upperRed = new Scalar(255, 100, 130);

                    Scalar lowerBlue = new Scalar(150, 50, 50);
                    Scalar upperBlue = new Scalar(255, 100, 130);

                    /**
                    Scalar lowerBlue = new Scalar(50, 50, 150);
                    Scalar upperBlue = new Scalar(130, 100, 255);
                     */

                    // Create a binary mask for red regions
                    Mat mask = new Mat();
                    Mat mask2 = new Mat();
                    Core.inRange(hsvImage, lowerRed, upperRed, mask);
                    Core.inRange(hsvImage, lowerBlue, upperBlue, mask2);

                    // Find contours in the binary mask
                    List<MatOfPoint> contours = new ArrayList<>(), contours2 = new ArrayList<>();
                    Mat hierarchy = new Mat();
                    Mat hierarchy2 = new Mat();
                    Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                    Imgproc.findContours(mask2, contours2, hierarchy2, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


                    // Find the contour with the largest area (zone with the most red)
                    maxArea = 0;
                    MatOfPoint maxContour = null;
                    for (MatOfPoint contour : contours) {
                        double area = Imgproc.contourArea(contour);
                        if (area > maxArea) {
                            maxArea = area;
                            maxContour = contour;
                        }
                    }

                    for (MatOfPoint contour : contours2) {
                        double area = Imgproc.contourArea(contour);
                        if (area > maxArea) {
                            maxArea = area;
                            maxContour = contour;
                        }
                    }

                    // Draw a circle around the identified zone
                    if (maxContour != null) {
                        Moments moments = Imgproc.moments(maxContour);
                        cx = (int) (moments.get_m10() / moments.get_m00());
                        cy = (int) (moments.get_m01() / moments.get_m00());

                        int radius = 100; // You can adjust the radius as needed
                        Imgproc.circle(input, new Point(cx, cy), radius, new Scalar(0, 255, 0), 2);
                    }
                    zoneFound = true;
                    // Return the processed frame

                }

                Imgproc.circle(input, new Point(cx, cy), 100, new Scalar(0, 255, 0), 2);
                if (cx <= 426)
                    pozitie = 1;
                else
                if (cx <= 852)
                    pozitie = 2;
                else
                    pozitie = 3;
                return input;


            }


            @Override
            public void onViewportTapped() {
                /*
                 * The viewport (if one was specified in the constructor) can also be dynamically "paused"
                 * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
                 * when you need your vision pipeline running, but do not require a live preview on the
                 * robot controller screen. For instance, this could be useful if you wish to see the live
                 * camera preview as you are initializing your robot, but you no longer require the live
                 * preview after you have finished your initialization process; pausing the viewport does
                 * not stop running your pipeline.
                 *
                 * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
                 */

                viewportPaused = !viewportPaused;

                if (viewportPaused) {
                    webcam.pauseViewport();
                } else {
                    webcam.resumeViewport();
                }
            }
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new SamplePipeline());

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
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
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addData("Arie", maxArea);
        telemetry.addData("cx", cx);
        telemetry.addData("pozitia este ", pozitie);
        telemetry.update();
        Initialise();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0);

        drive.setPoseEstimate(startPose);
        SCutie.setPosition(0.36);
        Trajectory tfirst = drive.trajectoryBuilder(startPose)
                .forward(27)
                .build();

        TrajectorySequence t2 = drive.trajectorySequenceBuilder(tfirst.end())
                .lineTo(new Vector2d(32.8, 0))
                .addDisplacementMarker(() -> {
                    SPixel.setPosition(0.4);
                })
                .lineTo(new Vector2d(50, 1))
                .lineTo(new Vector2d(50, 50))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(24.0, 50)) //29 pentru stanga
                .lineTo(new Vector2d(24.0, 83.25)) //81.75 //83.25
                        //SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence t3 = drive.trajectorySequenceBuilder(tfirst.end())
                .lineTo(new Vector2d(28, -11.1))
                .addDisplacementMarker(() -> {
                    SPixel.setPosition(0.4);
                })
                .lineTo(new Vector2d(28, 0))
                .lineTo(new Vector2d(50, 1))
                .lineTo(new Vector2d(50, 50))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(31.7, 50)) //29 pentru stanga
                .lineTo(new Vector2d(31.7, 83.25))
                      //  SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      //  SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(tfirst.end())
                .lineTo(new Vector2d(27, 13))
                .addDisplacementMarker(() -> {
                    SPixel.setPosition(0.4);
                })
                .lineTo(new Vector2d(27, 0))
                .lineTo(new Vector2d(50, 1))
                .lineTo(new Vector2d(50, 50))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(19, 50)) //29 pentru stanga
                .lineTo(new Vector2d(19, 83.25))
                     //   SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                     //   SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence ParkingMaster1_1 = drive.trajectorySequenceBuilder(t1.end())
                .lineTo(new Vector2d (3, 75))
                .lineTo(new Vector2d (3, 100))
                .build();

        TrajectorySequence ParkingMaster1_2 = drive.trajectorySequenceBuilder(t1.end())
                .lineTo(new Vector2d (52, 75))
                .lineTo(new Vector2d (52, 100))
                .build();

        TrajectorySequence ParkingMaster2_1 = drive.trajectorySequenceBuilder(t2.end())
                .lineTo(new Vector2d (3, 75))
                .lineTo(new Vector2d (3, 100))
                .build();

        TrajectorySequence ParkingMaster2_2 = drive.trajectorySequenceBuilder(t2.end())
                .lineTo(new Vector2d (52, 75))
                .lineTo(new Vector2d (52, 100))
                .build();

        TrajectorySequence ParkingMaster3_1 = drive.trajectorySequenceBuilder(t3.end())
                .lineTo(new Vector2d (3, 75))
                .lineTo(new Vector2d (3, 100))
                .build();

        TrajectorySequence ParkingMaster3_2 = drive.trajectorySequenceBuilder(t3.end())
                .lineTo(new Vector2d (52, 75))
                .lineTo(new Vector2d (52, 100))
                .build();

        TrajectorySequence reset = drive.trajectorySequenceBuilder(t2.end())
                .addTemporalMarker(5, () -> {})
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();

        waitForStart();

        webcam.stopRecordingPipeline();
        webcam.stopStreaming();



        pozitie = 3;
       // sleep(2000);
        if (pozitie == 2 || pozitie == 0)
        {
            drive.followTrajectory(tfirst);
            drive.followTrajectorySequence(t2);
            mb1.setTargetPosition(BratArray[4]);
            mb2.setTargetPosition(BratArray[4]);
            mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb1.setPower(0.17);
            mb2.setPower(0.17);
            SCutie.setPosition(SCutieArray[2]);
            while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                telemetry.update();
            }
            mb1.setPower(0);
            mb2.setPower(0);
            //drive.followTrajectorySequence(Wait2);
            sleep(300);
            SCutie.setPosition(SCutieArray[3]);
            sleep(1800);
            //SCutie.setPosition(0.34);
            //drive.followTrajectorySequence(Wait2);
            //sleep(1500);
            mb1.setTargetPosition(BratArray[1]);
            mb2.setTargetPosition(BratArray[1]);
            mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb1.setPower(0.2);
            mb2.setPower(0.2);
            SCutie.setPosition(SCutieArray[1]);
            while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                telemetry.update();
            }
            SCutie.setPosition(0.36);
            drive.followTrajectorySequence(ParkingMaster2_2);
        }
        else if (pozitie == 1)
        {
            drive.followTrajectory(tfirst);
            drive.followTrajectorySequence(t1);
            mb1.setTargetPosition(BratArray[4]);
            mb2.setTargetPosition(BratArray[4]);
            mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb1.setPower(0.17);
            mb2.setPower(0.17);
            SCutie.setPosition(SCutieArray[2]);
            while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                telemetry.update();
            }
            mb1.setPower(0);
            mb2.setPower(0);
            //drive.followTrajectorySequence(Wait2);
            sleep(300);
            SCutie.setPosition(SCutieArray[3]);
            sleep(1800);
            //SCutie.setPosition(0.34);
            //drive.followTrajectorySequence(Wait2);
            //sleep(1500);
            mb1.setTargetPosition(BratArray[1]);
            mb2.setTargetPosition(BratArray[1]);
            mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb1.setPower(0.2);
            mb2.setPower(0.2);
            SCutie.setPosition(SCutieArray[1]);
            while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                telemetry.update();
            }
            SCutie.setPosition(0.36);
            drive.followTrajectorySequence(ParkingMaster1_2);
        }
        else if (pozitie == 3)
        {
            drive.followTrajectory(tfirst);
            drive.followTrajectorySequence(t3);
            mb1.setTargetPosition(BratArray[4]);
            mb2.setTargetPosition(BratArray[4]);
            mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb1.setPower(0.17);
            mb2.setPower(0.17);
            SCutie.setPosition(SCutieArray[2]);
            while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                telemetry.update();
            }
            mb1.setPower(0);
            mb2.setPower(0);
            //drive.followTrajectorySequence(Wait2);
            sleep(300);
            SCutie.setPosition(SCutieArray[3]);
            sleep(1800);
           // SCutie.setPosition(0.34);
            //drive.followTrajectorySequence(Wait2);
            //sleep(1500);
            mb1.setTargetPosition(BratArray[1]);
            mb2.setTargetPosition(BratArray[1]);
            mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mb1.setPower(0.2);
            mb2.setPower(0.2);
            SCutie.setPosition(SCutieArray[1]);
            while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                telemetry.update();
            }
            SCutie.setPosition(0.36);
            drive.followTrajectorySequence(ParkingMaster3_2);
        }

        while (opModeIsActive());
    }
}
