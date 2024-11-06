package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;


/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous
//@Disabled
public class PixelDetection extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    int check_pixel_at_position = 1;
    List<Recognition> pixels = null;

    @Override
    public void runOpMode() {
        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        int pixel_detected_at = 0;
        boolean is_pixel_detected = false;
        initTfod();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
                // Share the CPUªª.
                sleep(20);
                if (!is_pixel_detected && check_pixel_at_position == 1) {
                    cameraServo.setPosition(0.5);
                    sleep(2000);
                    pixels = detectPixels();
                    if (pixels != null && pixels.size() >= 1) {
                        pixel_detected_at = 1;
                        is_pixel_detected = true;
                        telemetry.addData("# Objects Detected ", pixels.size());
                        telemetry.addData("# Pixel detected at", pixel_detected_at);
                        telemetry.addData("Servo Position ",cameraServo.getPosition());
                        telemetry.update();
                        sleep(8000);

                    } else {
                        telemetry.addData("# Objects Detected ","no object detected at position 1");
                    }
                    check_pixel_at_position = 3;
                }
                if (!is_pixel_detected && check_pixel_at_position == 3) {
                    cameraServo.setPosition(0.8);
                    pixels = detectPixels();
                    if (pixels != null && pixels.size() >= 1) {
                        is_pixel_detected = true;
                        pixel_detected_at = 3;
                        telemetry.addData("# Objects Detected", pixels.size());
                        telemetry.addData("# Pixel detected at ", pixel_detected_at);
                        telemetry.addData("Servo Position ",cameraServo.getPosition());
                        telemetry.update();
                        sleep(8000);
                    }  else {
                        telemetry.addData("# Objects Detected ","no object detected at position 3");
                    }
                    check_pixel_at_position = 3;
                }
                if (!is_pixel_detected) {
                    pixel_detected_at = 2;
                    is_pixel_detected = true;
                    telemetry.addData("# Pixel detected at ", pixel_detected_at);
                    telemetry.addData("Servo Position ",cameraServo.getPosition());
                    telemetry.update();
                    sleep(8000);

                }
                telemetry.update();
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private List<Recognition> detectPixels() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        return currentRecognitions;
    }   // end method telemetryTfod()

}   // end class


