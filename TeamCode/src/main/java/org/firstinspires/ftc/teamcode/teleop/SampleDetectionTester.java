package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SampleDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;

@TeleOp(name = "Sample Detection")
public class SampleDetectionTester extends LinearOpMode {
    SampleDetection sd;

    @Override
    public void runOpMode() throws InterruptedException {

        sd = new SampleDetection(telemetry);


        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(sd)
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();
        while(opModeIsActive())
        {

        }


    }
}
