package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class DetectPixel {
    static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    TfodProcessor tfod;
    VisionPortal visionPortal;

    Controller controller;

    public DetectPixel(Controller controller, HardwareMap hardwareMap) {
        this.controller = controller;

        initTfod(hardwareMap);
    }

    public void initTfod(HardwareMap hardwareMap) {
        tfod = TfodProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, tfod);
        }
    }

    public void closeTfod() {
        visionPortal.close();
    }

    public double getDistance() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        if (currentRecognitions.size() > 0) {
            Recognition closestRecognition = currentRecognitions.get(0);

            double x = (closestRecognition.getLeft() + closestRecognition.getRight()) / 2;
            double y = (closestRecognition.getTop()  + closestRecognition.getBottom()) / 2;

            return y;
        }

        return -1;
    }

    public void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        controller.Debug("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2;

            controller.Debug("", " ");
            controller.Debug("Image", (recognition.getLabel() + " (" + recognition.getConfidence() * 100 + "% Conf.)"));
            controller.Debug("- Position", (x + " / " + y));
            controller.Debug("- Size", (recognition.getWidth() + " x " + recognition.getHeight()));
        }
    }
}