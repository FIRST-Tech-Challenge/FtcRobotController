package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        Pose2d startPose = new Pose2d(-60, 40, 0);
        drive.setPoseEstimate(startPose);
        TrajectorySequence aSeq = autoSeq(startPose);
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
                //if (currTime - startTime < 500) {
                //    telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
                //} else {
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
                //}
                telemetry.update();
                currTime = System.currentTimeMillis();

            }

        }
        telemetry.update();
        done = false;

        //lift arm up
        actuatorUtils.armPole(4);
        while (((currTime - startTime) < 30000) && !done && opModeIsActive()) {
            drive.followTrajectorySequence(aSeq);
            drive.followTrajectorySequence(parkSeq(resultROI));
            currTime = System.currentTimeMillis();
            done = true;
        }
    }

    private TrajectorySequence autoSeq(Pose2d pose) {
        TrajectorySequence seq = drive.trajectorySequenceBuilder(pose)
                .lineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(-90)))
                .forward(24)
                .turn(Math.toRadians(90))
                .forward(24)
                .turn(Math.toRadians(-45))
                .forward(9)
                .waitSeconds(2)
                //.addTemporalMarker(5, () -> {
                //    try {
                //        actuatorUtils.armPole(1, true);
                //        actuatorUtils.gripperOpen(true);
                //        actuatorUtils.armPole(4, true);
                //   }
                //    catch (InterruptedException ex) {
                //        telemetry.addData(ex.getLocalizedMessage(), "");
                //        telemetry.update();
                //    }
                //})
                .forward(-9)
                .turn(Math.toRadians(45))
                .forward(24)
                .turn(Math.toRadians(90))
                .build();
        return seq;
    }
    private TrajectorySequence parkSeq(int park) {
        TrajectorySequence seq = drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(((park-1)*-12))
                    .build();
        return seq;
    }
    private void initOpenCV() {
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


