package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@TeleOp
public class AprilTagsDetection extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagsPipeline AprilTagsPipeline;

    static final double FEET_PER_METER = 3.28084;

    //will have to change this with the webcam
    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

    // UNITS ARE METERS
    //might have to change this? not sure what size the tags will be
    double tagsize = 0.166;

    int ONE = 1;
    int TWO = 2;
    int THREE = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagsPipeline = new org.firstinspires.ftc.teamcode.AprilTagsPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(AprilTagsPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //init loop
        while (!isStarted() && !isStopRequested()){
            ArrayList<AprilTagDetection> currentDetections = AprilTagsPipeline.getLatestDetections();

            if(currentDetections.size() != 0){
                boolean tagFound = false;
                for(AprilTagDetection tag : currentDetections){
                    if(tag.id == ONE || tag.id == TWO || tag.id == THREE){
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound){
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else{
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null){
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else{
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else{
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null){
                    telemetry.addLine("(The tag has never been seen)");
                }
                else{
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        //update telemetry
        if(tagOfInterest != null){
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else{
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        if(tagOfInterest == null){
            //default auto for no detection
        } else if(tagOfInterest.id == ONE){
            //to april tag 1
        } else if(tagOfInterest.id == TWO){
            //to april tag 2
        } else {
            //to april tag 3
        }
        {


        }


    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}