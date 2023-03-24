package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "LeftAutoComp", group = "Taus2022-23")
public class LeftCompCode extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();
    private Vector2d myVector = new Vector2d(10, -5);
    private Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));
    private SampleMecanumDrive drive;
    private ManipulatorMethods manipulator;

    int START_TICKS = (int)(271/1.5);
    int INTAKE_TICKS = 0;
    int LOW_TICKS = (int)(2063/1.5);
    int MID_TICKS = (int)(3300/1.5);
    int HIGH_TICKS = (int)(4900/1.5);


    static final double FEET_PER_METER = 3.28084;


    //CVision
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;

    AprilTagDetection tagOfInterest = null;
    int tagNum = 0;


    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        manipulator = new ManipulatorMethods(hardwareMap);
        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("Waiting for start");
        telemetry.update();
        timer.reset();

        drive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(90)));

        telemetry.addLine("SetPose");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagNum = tag.id;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }


            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        Trajectory firstMidPole = drive.trajectoryBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, -18, Math.toRadians(315)), Math.toRadians(315))
                .build();
        Trajectory stackMid = drive.trajectoryBuilder(firstMidPole.end())
                .lineTo(new Vector2d(-36, -12))
                .build();
        Trajectory stackMid2 = drive.trajectoryBuilder(stackMid.end())
                .lineToLinearHeading(new Pose2d(-35, -12, Math.toRadians(180)))
                .build();
        Trajectory stack = drive.trajectoryBuilder(stackMid2.end())
                .lineTo(new Vector2d(-63, -10))
                .build();
        Trajectory poleMid = drive.trajectoryBuilder(stack.end())
                .lineTo(new Vector2d(-12, -12))
                .build();
        Trajectory pole = drive.trajectoryBuilder(poleMid.end())
                .lineToLinearHeading(new Pose2d(-19, -19, Math.toRadians(225)))
                .build();
        Trajectory poleBack = drive.trajectoryBuilder(pole.end())
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .build();
        Trajectory stack2 = drive.trajectoryBuilder(poleBack.end())
                .lineTo(new Vector2d(-63, -10.15))
                .build();
        Trajectory poleMid2 = drive.trajectoryBuilder(stack2.end())
                .lineTo(new Vector2d(-12, -12))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(180)))
                .build();
        Trajectory park2 = drive.trajectoryBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                .build();
        Trajectory park3 = drive.trajectoryBuilder(stack2.end())
                .lineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(180)))
                .build();
        waitForStart();

        waitForStart();
        telemetry.addLine("Start requested");
        telemetry.update();

        if(isStopRequested()) return;

        manipulator.resetSlides();
        manipulator.moveSlideEncoder(MID_TICKS, 1);


        drive.followTrajectory(firstMidPole);
        manipulator.outtake();
        sleep(1500);
        manipulator.stopIntake();

        drive.followTrajectory(stackMid);
        drive.followTrajectory(stackMid2);
        manipulator.moveSlideEncoder(LOW_TICKS-600, 1);

        drive.followTrajectory(stack);
        manipulator.moveSlideEncoder(LOW_TICKS - 850, 1);
        manipulator.intake();
        sleep(750);
        manipulator.stopIntake();
        manipulator.moveSlideEncoder(LOW_TICKS, 1);
        sleep(250);

        drive.followTrajectory(poleMid);
        manipulator.moveSlideEncoder(MID_TICKS, 1);

        drive.followTrajectory(pole);

        manipulator.outtake();
        sleep(1000);
        manipulator.stopIntake();

        drive.followTrajectory(poleBack);
        manipulator.moveSlideEncoder(LOW_TICKS-700, 1);

        drive.followTrajectory(stack2);
        manipulator.moveSlideEncoder(LOW_TICKS - 1050, 1);
        manipulator.intake();
        sleep(750);
        manipulator.stopIntake();
        manipulator.moveSlideEncoder(MID_TICKS, 1);
        sleep(250);

        drive.followTrajectory(poleMid2);
        drive.followTrajectory(pole);

        manipulator.outtake();
        sleep(1000);
        manipulator.stopIntake();

        drive.followTrajectory(poleBack);
        manipulator.moveSlideEncoder(LOW_TICKS-900, 1);

        drive.followTrajectory(stack2);
        manipulator.moveSlideEncoder(LOW_TICKS - 1250, 1);
        manipulator.intake();
        sleep(750);
        manipulator.stopIntake();
        manipulator.moveSlideEncoder(LOW_TICKS, 1);
        sleep(500);
        if (tagNum == 0) {
            drive.followTrajectory(park1);
            manipulator.moveSlideEncoder(INTAKE_TICKS, 1);
            sleep(750);
        } else if (tagNum == 1) {
            drive.followTrajectory(park2);
            manipulator.moveSlideEncoder(INTAKE_TICKS, 1);
            sleep(750);
        } else {
            drive.followTrajectory(park3);
            manipulator.moveSlideEncoder(INTAKE_TICKS, 1);
            sleep(750);

        }
        PoseHolder.slideHeight = manipulator.rightSlides.getCurrentPosition();

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}

