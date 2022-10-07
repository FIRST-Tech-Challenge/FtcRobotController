package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

public class ParkingPositionUtil {
    Telemetry telemetry;
    private OpenCvWebcam webcam;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

    public ParkingPositionUtil(HardwareMap hardwareMap, String webcamName, Telemetry telemetry){
        this.telemetry = telemetry;
        setup(hardwareMap, webcamName);
    }

    public void setup(HardwareMap hardwareMap,String webcamName){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
    }

    public void init(){
        openCameraDevice();
    }

    public void setTimeoutTime( int milliseconds ) {
        // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout( milliseconds );
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

    public AprilTagDetection DetectTag(){
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == 13 || tag.id == 0 || tag.id == 19)
                {
                    tagOfInterest = tag;
                    return tagOfInterest;
                }
            }
        }
        return tagOfInterest;
    }

    public String getParkingPosition(){
        AprilTagDetection tag = DetectTag();
        String parkingPosition = null;

        if(tag.id == 13){
            parkingPosition= "LEFT";
        }
        else if(tag.id == 0){
            parkingPosition= "MIDDLE";
        }
        else if(tag.id == 19){
            parkingPosition= "RIGHT";
        }

        return parkingPosition;
    }
}
