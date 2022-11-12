package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoRedRight extends LinearOpMode {

    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    OpenCvCamera camera;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    //don't change
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //our three tags
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLD();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("no tag boi");
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");


            }

            telemetry.update();
            sleep(20);
        }


        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, never sighted(");
            telemetry.update();
        }
        TrajectorySequence seq = null;
        Pose2d pos = new Pose2d(35.5, 64.75, Math.toRadians(270));
        robot.setPoseEstimate(pos);
        if (tagOfInterest.id == LEFT) {
            //insert trajectories for parking zone 1
            seq = robot.trajectorySequenceBuilder(pos)
                    .forward(17.5)
                    .turn(Math.toRadians(-90))
                    .forward(4)
                    //use claw
                    .build();
            robot.servo.setPosition(0);
            seq = robot.trajectorySequenceBuilder(pos)
                    .back(4)
                    .turn(Math.toRadians(-90))
                    .forward(17.5)
                    .turn(Math.toRadians(-90))
                    .forward(25)
                    .turn(Math.toRadians(-90))
                    .forward(25)
                    .build();
        }

        else if (tagOfInterest.id == MIDDLE) {
            //insert trajectories for parking zone 2
            seq = robot.trajectorySequenceBuilder(pos)
                    .forward(17.5)
                    .turn(Math.toRadians(-90))
                    .forward(4)
                    .build();
                    //use claw
            robot.servo.setPosition(0);
            seq = robot.trajectorySequenceBuilder(pos)
                    .back(4)
                    .turn(Math.toRadians(-90))
                    .forward(17.5)
                    .turn(Math.toRadians(90))
                    .forward(25)
                    .turn(Math.toRadians(90))
                    .forward(50)
                    .turn(Math.toRadians(90))
                    .forward(30)
                    .build();
        }

        else if (tagOfInterest.id == RIGHT) {
            //insert trajectories for parking zone 3
            seq = robot.trajectorySequenceBuilder(pos)
                    .forward(17.5)
                    .turn(Math.toRadians(-90))
                    .forward(4)
                    //use claw
                    .build();
            robot.servo.setPosition(0)
            seq = robot.trajectorySequenceBuilder(pos)
                    .back(4)
                    .turn(Math.toRadians(-90))
                    .forward(17.5)
                    .turn(Math.toRadians(90))
                    .forward(25)
                    .turn(Math.toRadians(90))
                    .forward(50)
                    .turn(Math.toRadians(90))
                    .build();
        }
        waitForStart();
        if(!isStopRequested() && seq != null){
            robot.followTrajectorySequence(seq);
        }


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

    }
}