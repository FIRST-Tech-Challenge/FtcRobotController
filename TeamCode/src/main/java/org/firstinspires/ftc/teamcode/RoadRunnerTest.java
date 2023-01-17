package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "RoadRunnerTest", group = "")
public class RoadRunnerTest extends LinearOpMode {
    //test1

    private SampleMecanumDrive drive;
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    private DcMotor arm = null;

    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public double desiredHeading;

    Orientation angles;
    Acceleration gravity;
    private OpenCvCamera webCam;
    private boolean isCameraStreaming = false;
    Pipeline modifyPipeline = new Pipeline();

    private int resultROI = 2;

    private boolean done = false;
/*
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);

        desiredHeading = getHeading();

        actuatorUtils.initializeActuator(arm, gripper);


        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        initOpenCV();

        actuatorUtils.gripperClose(false);

        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        if (resultROI == 2) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            done = false;
            while (!done && opModeIsActive()) {
                if (currTime - startTime < 500) {
                    telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
                } else {
                    telemetry.addData("Time Delta: ", (currTime - startTime));
                    resultROI = modifyPipeline.getResultROI();
                    if (resultROI == 1) {
                        telemetry.addData("Resulting ROI: ", "Red");
                        done = true;
                    } else if (resultROI == 2) {
                        telemetry.addData("Resulting ROI: ", "Green");
                        done = true;
                    } else if (resultROI == 3) {
                        telemetry.addData("Resulting ROI: ", "Blue");
                        done = true;
                    } else {
                        telemetry.addData("Resulting ROI: ", "Something went wrong.");
                    }
                }
                telemetry.update();
                currTime = System.currentTimeMillis();

            }

        }
        telemetry.update();
        done = false;
        //lift arm up
        actuatorUtils.armPole(4);
        while (((currTime - startTime) < 30000) && !done && opModeIsActive()) {

            switch (resultROI) {
                case 1:
                    telemetry.addData("Strafing", "case 1");
                    telemetry.update();
                    done = true;
                    break;
                case 2:
                    done = true;
                    break;
                case 3:
                    // Far right
                    telemetry.addData("Executing", "case 3");
                    telemetry.update();
                    done = true;
                    break;
            }

            currTime = System.currentTimeMillis();

        }

         */
        public static double DISTANCE = 48; // in

        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

            drive.setPoseEstimate(startPose);

            waitForStart();

            if (isStopRequested()) return;

            while (!isStopRequested()) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                        .lineTo(new Vector2d (DISTANCE, 0))
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d (DISTANCE, DISTANCE))
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d (0, DISTANCE))
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d (0, 0))
                        .turn(Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(trajSeq);
            }
        }
    }


    /*private void initOpenCV() {
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        // For a webcam (uncomment below)
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId2);
        // For a phone camera (uncomment below)
        // webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Pipeline: ", "Initialized");
                telemetry.update();
                isCameraStreaming = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getHeading() {
        double angle = drive.getRawExternalHeading();
        return angle;
    }

}
*/

