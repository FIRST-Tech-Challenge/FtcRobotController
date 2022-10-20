package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp
public class AutonRight extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetect;

    static final double FEET_PER_METER = 3.28084;

    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448. WE MUST CHANGE THESE TO MATCH OUR CONFIGURATIONS

    /*
    Check constructMatrix in apriltagdetectionpipeline file for more information

    */
    double fx = 369.50;
    double fy = 369.50;
    double cx = 320;
    double cy = 240;
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.0406;
//    double tagsize = 0.166;

    final int ID_LEFT = 0; // RANDOM TAG CHOSEN
    final int ID_MIDDLE = 1;
    final int ID_RIGHT = 2;
    int parkingSpace;

    AprilTagDetection tagOfInterest = null;

    int tagCount = 0;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetect = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetect);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * This REPLACES waitForStart
         */

        boolean continueAuton = true;
        while (!isStarted() && !isStopRequested() && continueAuton)
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetect.getLatestDetections();
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    telemetry.addLine(String.format("\ntagid", tag.id));
                    if(tag.id == ID_LEFT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if(tag.id == ID_MIDDLE)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if(tag.id == ID_RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    processTag(tagOfInterest);
                    break;
                }
                else
                {
                    telemetry.addLine("Tag of interest not in sight");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        processTag(tagOfInterest);
                        break;
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
                    processTag(tagOfInterest);
//                    break;

                }

            }

            telemetry.update();
            sleep(20);
        }



        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Well then, something went wrong!
             */

            telemetry.addLine("Could not find the tag...");
            telemetry.update();
        }
        else {
            waitForStart();

            if (tagCount == 1) {
                Drive drive = new Drive(hardwareMap);
                telemetry.addData("Parking Space", parkingSpace);
                telemetry.update();
                Auton auto = new Auton(false);
                auto.runAuton(parkingSpace, drive);
            }
        }



        /* Prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void processTag(AprilTagDetection detection)
    {
        /*
         * this is where we run the auton
         */

        // three different tags:
        switch(detection.id) {
            case ID_LEFT:
                telemetry.addLine("FOUND LEFT");
                break;
            case ID_MIDDLE:
                telemetry.addLine("FOUND MIDDLE");
                break;
            case ID_RIGHT:
                telemetry.addLine("FOUND RIGHT");
                break;
        }


        if (detection.id == 1) { parkingSpace = 1; }
        else if (detection.id == 0) { parkingSpace = 2; }
        else { parkingSpace = 3; }

        tagCount++;

        telemetry.update();

        telemetry.addLine(String.format("\nDetected tag ID=%d", parkingSpace));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}