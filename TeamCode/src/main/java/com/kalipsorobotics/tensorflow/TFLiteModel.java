/*
package org.firstinspires.ftc.teamcode.NewStuff;

import com.kalipsorobotics.fresh.Vision;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "TFLiteModel", group = "")

public class TFLiteModel extends LinearOpMode {

    //loading in the model by giving the computer the path name to the model
    private static final String MODEL_PATH = "/Users/chloewu/kalipso/TF_Detection/model.tflite";

    // private static final String MODEL_NAME = "model.tflite";
    private static final String[] MODEL_LABELS = {
            "Pixel",
    };

    //declaring variables
    private static final boolean USE_WEBCAM = true;
    private TfodProcessor Tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        //initiating Tfod (see code below)
        initTfod();

        //what happens during init - displays text
        telemetry.addData("DS preview on/off","3 dots, Camera Stream");
        telemetry.addData(">","Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        //after pressing start, (opMode) we use telemetryTfod (check code below) - stopstreaming vs resume streaming using gamepad
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                telemetry.update();

                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                sleep(20);
            }
        }
            visionPortal.close(); //save energy
    }

    //basically we need to tell the computer what initTfod which is what this chunk of code does
    private void initTfod() {

        //loading the model with Tfod Processor (initTfod)
        Tfod = new TfodProcessor.Builder()
                .setModelFileName(MODEL_PATH)
                .setModelLabels(MODEL_LABELS)
                .build();

        //telling computer what and which webcam to use
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), Tfod);
          } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, Tfod);
        }
    }

    //telling computer what "telemetryTfod" means and what it needs to display on the driver hub
    private void telemetryTfod(){
        List <Recognition> currentRecognitions = Tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for(Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) /2;
            double y = (recognition.getTop() + recognition.getBottom()) /2;

            telemetry.addData(" ", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

        }
    }
}






*/
