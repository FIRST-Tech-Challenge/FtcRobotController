package org.firstinspires.ftc.teamcode.robotSubSystems.Camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class cameraTest extends LinearOpMode {
VisionPortal portal;
ObjectAngleCalculator angleCalculator = new ObjectAngleCalculator();

    @Override
    public void runOpMode() throws InterruptedException {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(angleCalculator)
                .build();

        waitForStart();

        while (!isStopRequested()){
            telemetry.addData("angle",ObjectAngleCalculator.angle);
            telemetry.update();
        }


    }
}
