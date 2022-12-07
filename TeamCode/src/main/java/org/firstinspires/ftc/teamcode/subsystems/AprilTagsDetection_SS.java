package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.openCV.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AprilTagsDetection_SS  extends SubsystemBase {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private final Telemetry mTelemetry;


    static final double CM_PER_METER = 100.00;



 //Logitech HD Webcam C270, Calibration size="640 480"
// voir fichier XML dans dossier  res/xml
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;


    // UNITS ARE METERS
    double tagsize = 0.166;


    //DBLISS Custom tags
    int zone1 = 1;
    int zone2 = 2;
    int zone3 = 3;
    AprilTagDetection tagOfInterest = null;




    public AprilTagsDetection_SS (HardwareMap hardwareMap, Telemetry telemetry) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        mTelemetry = telemetry;

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



    }



    @Override
    public void periodic() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == zone1 || tag.id == zone2 || tag.id == zone3)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                mTelemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                mTelemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    mTelemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    mTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            mTelemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                mTelemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                mTelemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        //mTelemetry.update();
       // mTelemetry.addData("Tag détecté", tagOfInterest.id);
        mTelemetry.addData("Tag détecté_ModeAuto:", readTags());
    }


    public int readTags()
    {
        int tagDetected;

        if( tagOfInterest == null )
        {
            //code pour coneGauche trajectoire
            tagDetected = 0;
           return tagDetected;

        } else if (tagOfInterest.id == zone1) {

            tagDetected = 1;
            return tagDetected;

        }  else if (tagOfInterest.id == zone2) {

            tagDetected = 2;
            return tagDetected;

        }  else if (tagOfInterest.id == zone3) {

            tagDetected = 3;
            return tagDetected;

        }else {

            //code pour trajectoire droite
           tagDetected = 0;
            return tagDetected;
        }


    }


    /*
    public int readTags2() // originalCode
    {



        if(tagOfInterest != null)
        {
            mTelemetry.addLine("Tag identification:\n");
            tagToTelemetry(tagOfInterest);
            mTelemetry.update();
        }
        else
        {
            mTelemetry.addLine("pas de Tags disponible, non vu durant le init loop :(");
            mTelemetry.update();
        }



        if(tagOfInterest == null ||  tagOfInterest.id == zone1)
        {
            //code pour coneGauche trajectoire
            return tagOfInterest.id;

        } else if (tagOfInterest.id == zone2) {

            //code pour coneGauche trajectoire
            return tagOfInterest.id;

        }  else if (tagOfInterest.id == zone3) {

            //code pour coneGauche trajectoire
            return tagOfInterest.id;

        }else {

            //code pour trajectoire droite
            return tagOfInterest.id ;
        }


    }

*/



    void tagToTelemetry(AprilTagDetection detection)
    {
        mTelemetry.addLine(String.format("\ndetection tag ID=%d", detection.id));
        mTelemetry.addLine(String.format("Translation X: %.2f CM", detection.pose.x*CM_PER_METER));
        mTelemetry.addLine(String.format("Translation Y: %.2f CM", detection.pose.y*CM_PER_METER));
        mTelemetry.addLine(String.format("Translation Z: %.2f CM", detection.pose.z*CM_PER_METER));
        mTelemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        mTelemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        mTelemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }









}





