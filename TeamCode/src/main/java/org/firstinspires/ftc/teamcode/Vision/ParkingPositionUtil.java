package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class ParkingPositionUtil {
    Telemetry telemetry;
    OpenCvCamera webcam;
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
    double tagsize = 0.166;

    // Tag ID 18 from the 36h11 family
    int LEFT = 13;
    int MIDDLE = 0;
    int RIGHT = 19;

    AprilTagDetection tagOfInterest = null;

    public String ParkingPosition;

    public ParkingPositionUtil(HardwareMap hardwareMap, String webcamName, Telemetry telemetry){
        this.telemetry = telemetry;
        setup(hardwareMap, webcamName);
    }

    public void setup(HardwareMap hardwareMap,String name){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);

        openCameraDevice();
    }

    public void init(){
        openCameraDevice();
    }

    public void openCameraDevice( ) {

        webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
            @Override
            public void onOpened( ) {
                webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
            }

            @Override
            public void onError( int errorCode ) {
                //This will be called if the camera could not be opened
                telemetry.addLine( "Camera could not be opened. Error code: " + errorCode );
            }
        } );
    }

    public void stopCamera( ) {
        webcam.stopStreaming( );
    }

    public void DetectTag() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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
                telemetry.addLine("Tag of interest is in sight!");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        /* Actually do something useful */
        if (tagOfInterest.id == LEFT) {
            //Default path is LEFT
            telemetry.addLine("Left Parking spot");
        } else if (tagOfInterest.id == MIDDLE) {
            telemetry.addLine("Middle Parking spot");
        } else if (tagOfInterest.id == RIGHT) {
            telemetry.addLine("Right Parking spot");
        }
    }


   /* public String getParkingPosition(){
        int tag = DetectTag();

        if(tag == 13){
            parkingPosition= "LEFT";
        }
        else if(tag == 0){
            parkingPosition= "MIDDLE";
        }
        else if(tag == 19){
            parkingPosition= "RIGHT";
        }

        return parkingPosition;
    }
*/
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
