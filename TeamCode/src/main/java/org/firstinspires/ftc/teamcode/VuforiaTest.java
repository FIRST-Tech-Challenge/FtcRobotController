package org.firstinspires.ftc.teamcode;

import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaLocalizerAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class VuforiaTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware map
        HardwareMap hardwareMap = this.hardwareMap;

        // Retrieve the webcam from the hardware map
        CameraName cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

//        // Initialize the VuforiaLocalizer with your license key
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        // Set the camera name and the camera direction
//        parameters.cameraName = cameraName;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        // Create a VuforiaLocalizer with the specified parameters
//        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Wait for the start button to be pressed
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Capture a frame from the webcam
//            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().poll();
//            if (frame != null) {
//                try {
//                    // Access the image from the frame
//                    Image image = frame.getImage(0); // 0 corresponds to the index of the color image
//
//                    // Process the captured frame here
//                    // You can access the image data using image.getPixels(), image.getWidth(), image.getHeight(), etc.
//
//                } finally {
//                    // Close the frame to release resources
//                    frame.close();
//                }
//            }
//        }
//
//        // Perform cleanup and end the op mode
//        vuforia.close();
    }
}
