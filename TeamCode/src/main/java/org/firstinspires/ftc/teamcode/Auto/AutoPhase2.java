package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="AutoPhase2")
public class AutoPhase2 extends LinearOpMode {

    Apriltag aprilTagProcessor = new Apriltag("blueTeam");
    Tfod tfodProcessor = new Tfod();
    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;

    private void initVision() {

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change name depending on mech team wants to name it
                .addProcessor(aprilTagProcessor.getAprilTagProcessor())
                .addProcessor(tfodProcessor.getTfodProcessor())
                .setCameraResolution(new Size(640, 480)) // import android.util.Size;
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // changed to MJPEG instead of YUY2 since it uses less bandwidth
                // .enableCameraMonitoring(true) // enableCameraMonitoring not identified? I also didn't find any related methods in VisionPortal.java
                .enableLiveView(true) // manually added this method because it was the closest thing to enableCameraMonitoring
                .setAutoStopLiveView(true)
                .build();

    }

    @Override
    public void runOpMode() {

    }


}

// Visionportal5 = new class of the apriltag code; make an object from it
// Visiportal6 = new class of the tfod code; make an object from it