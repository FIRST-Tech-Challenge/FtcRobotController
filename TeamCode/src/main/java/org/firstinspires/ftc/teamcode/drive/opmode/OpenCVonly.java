package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DefaultFunctions;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;

@Autonomous(name = "RRparkonly", group = "Concept")
public class OpenCVonly extends LinearOpMode
{
    private DefaultFunctions defaultFunctions;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double TagSize = 0.166;

    // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TagSize, fx, fy, cx, cy);

        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence LeftPark = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(0, 24))
                .lineTo(new Vector2d(36, 24))
                .build();

        TrajectorySequence RightPark = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(0, -16))
                .lineTo(new Vector2d(36, -16))
                .build();

        TrajectorySequence MiddlePark = drivetrain.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineTo(new Vector2d(36, 0))
                .build();

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

        telemetry.setMsTransmissionInterval(50);

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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
*/

        /* Update the telemetry*/
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            telemetry.addLine("Going for left");
            drivetrain.followTrajectorySequence(LeftPark);
        }else if(tagOfInterest.id == MIDDLE){
            telemetry.addLine("Going for centre");
            drivetrain.followTrajectorySequence(MiddlePark);
        }else if(tagOfInterest.id == RIGHT){
            telemetry.addLine("Going for right");
            drivetrain.followTrajectorySequence(RightPark);
        }
    }

    void tagToTelemetry(@NonNull AprilTagDetection detection)
    {
        telemetry.addLine(String.format(Locale.ENGLISH,"Detected tag ID=%d", detection.id));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format(Locale.ENGLISH,"Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}