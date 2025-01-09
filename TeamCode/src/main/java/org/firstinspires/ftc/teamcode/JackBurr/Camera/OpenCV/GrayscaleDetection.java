package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV.SampleDetectorPipelineGreyscale;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@TeleOp
public class GrayscaleDetection extends OpMode {

    private VisionPortal visionPortal;
    public boolean isBlue;
    private SampleDetectorPipelineGreyscale bad;
    public boolean setup = false;

    @Override
    public void init() {
        isBlue = true;
    }

    @Override
    public void init_loop(){
        while (!setup) {
            telemetry.addData("Press X for Blue, B for Red", "");
            if (gamepad1.x) {
                isBlue = true;
                telemetry.addData("Selected", "Blue");
            } else if (gamepad1.b) {
                isBlue = false;
                telemetry.addData("Selected", "Red");
            }
            telemetry.update();
            setup = true;
        }
        bad = new SampleDetectorPipelineGreyscale(isBlue);

        initVision();
    }

    @Override
    public void loop() {
        telemetry.addData("Brick Angle", bad.getAngle());
        telemetry.update();
        visionPortal.close();
    }

    public void initVision() {
         visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(bad)
                .build();
        visionPortal.setProcessorEnabled(bad, true);
    }
}