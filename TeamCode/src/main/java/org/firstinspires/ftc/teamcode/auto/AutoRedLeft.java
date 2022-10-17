package org.firstinspires.ftc.teamcode.auto;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCamera;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous
public class AutoRedLeft extends LinearOpMode {

    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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
    int left = 1;
    int middle = 2;
    int right = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        robot.camera.setPipeline(aprilTagDetectionPipeline);
        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                robot.camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLD();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == right || tag.id == middle)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("tag is spotted:");

                }
            }

            telemetry.update();
            sleep(15);
        }


        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available");
            telemetry.update();
        }

        //ADD TRAJECTORIES HERE
        //https://firstinspiresst01.blob.core.windows.net/first-energize-ftc/game-manual-part-2-traditional.pdf
        //page 46 above has the parking zones

        if(tagOfInterest.id == left )
        {
            //insert trajectories for parking zone 1
            //drive.followTrajectory(myTrajectory)
        }

        else if(tagOfInterest.id == middle)
        {
            //insert trajectories for parking zone 2
        }

        else if(tagOfInterest.id == right)
        {
            //insert trajectories for parking zone 3
        }


    }


}

