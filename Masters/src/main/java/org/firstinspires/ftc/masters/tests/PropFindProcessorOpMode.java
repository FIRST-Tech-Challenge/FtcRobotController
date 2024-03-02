package org.firstinspires.ftc.masters.tests;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.masters.PropFindProcessor;
import org.firstinspires.ftc.masters.PropFindRight;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous(name="Processor CV", group="cv")
public class PropFindProcessorOpMode extends OpMode {
    private VisionPortal visionPortal;
    private PropFindProcessor propFindProcessor;
    
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        propFindProcessor = new PropFindProcessor(
                telemetry,
                packet
        );
        
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "frontWebcam")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(propFindProcessor)
                .build();

        // you may also want to take a look at some of the examples for instructions on
        // how to have a switchable camera (switch back and forth between two cameras)
        // or how to manually edit the exposure and gain, to account for different lighting conditions
        // these may be extra features for you to work on to ensure that your robot performs
        // consistently, even in different environments
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", propFindProcessor.position);
        telemetry.addData("Camera State", visionPortal.getCameraState());
    }

    @Override
    public void start() {
        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        // gets the recorded prop position
        PropFindProcessor.pos recordedPropPosition = propFindProcessor.position;
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        // this closes down the portal when we stop the code, its good practice!
        propFindProcessor.close();
        visionPortal.close();
    }
}