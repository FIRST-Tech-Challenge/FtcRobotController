package org.firstinspires.ftc.masters.components;

import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.masters.PropFindLeftProcessor;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class ComputerVision {

    protected AprilTagProcessor aprilTag;
    protected PropFindRightProcessor propFindProcessor;
    protected VisionPortal myVisionPortal;
    private WebcamName frontWebcam, backWebcam;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    TelemetryPacket packet = new TelemetryPacket();

    private static final boolean USE_WEBCAM = true;

    public ComputerVision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    public void initializeAprilTagProcessing(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                .setTagLibrary((SkystoneDatabase.SkystoneDatabase()))
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
    }

    public void initializePropFindRightProcessing(){
        propFindProcessor = new PropFindRightProcessor(telemetry,packet);
    }

    public void initializePropFindLeftProcessing(){
        propFindProcessor = new PropFindLeftProcessor(telemetry,packet);
    }



    public void initializeVisionPortal(){

        frontWebcam = hardwareMap.get(WebcamName.class, "frontWebcam");
        backWebcam = hardwareMap.get(WebcamName.class, "backWebcam");

        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(frontWebcam, backWebcam);

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(switchableCamera)
                    .setCameraResolution(new Size(640, 360))
                    .addProcessors(propFindProcessor, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(propFindProcessor, aprilTag)
                    .build();
        }


    }

    public VisionPortal getMyVisionPortal() {
        return myVisionPortal;
    }

    public AprilTagProcessor getAprilTag() {
        return aprilTag;
    }

    public PropFindRightProcessor getPropFindProcessor() {
        return propFindProcessor;
    }

    public void activateFrontCamera (){

        myVisionPortal.getCameraState();
        myVisionPortal.setActiveCamera(frontWebcam);
    }

    public  void activateBackCamera(){
        myVisionPortal.setActiveCamera(backWebcam);
    }

    public void enableAprilTag(){
        myVisionPortal.setProcessorEnabled(aprilTag, true);
        myVisionPortal.setProcessorEnabled(propFindProcessor, false);
    }

    public void enablePropProcessor(){
        myVisionPortal.setProcessorEnabled(propFindProcessor, true);
        myVisionPortal.setProcessorEnabled(aprilTag, false);
    }

    public void stopPropProcessor(){
        myVisionPortal.setProcessorEnabled(propFindProcessor, false);
    }

}
