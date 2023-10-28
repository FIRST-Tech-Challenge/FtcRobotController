package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
public class AprilTagsDetection{

    static AprilTagsPipeline pipeline;

    static final double FEET_PER_METER = 3.28084;
    static final double PIXELS_PER_METER = 3779.5275591;

    //will have to change this with the webcam
    public double fx = 1078.03779;
    public double fy = 1084.50988;
    public double cx = 580.850545;
    public double cy = 245.959325;

    // UNITS ARE METERS
    //might have to change this? not sure what size the tags will be
    public double tagsize = 0.166;

    static int ONE = 1;
    static int TWO = 2;
    static int THREE = 3;


    static AprilTagDetection tagOfInterest = null;

    public static void detectTag(){

        telemetry.setMsTransmissionInterval(50);


            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

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
    }

     public double calcDistToTag(){
        detectTag();
        double distance = (tagsize * Math.sqrt(fx*fy))/(2*tagsize*PIXELS_PER_METER);
        return distance;
    }




    static void tagToTelemetry(AprilTagDetection detection)
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

class AprilTagException extends Exception{
    public AprilTagException(){
    }

    public AprilTagException(String message)
    {
        super(message);
    }
}