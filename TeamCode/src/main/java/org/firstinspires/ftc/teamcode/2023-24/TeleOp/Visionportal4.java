///* Copyright (c) 2019 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import android.util.Size;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
///*
// * This OpMode illustrates the basics of TensorFlow Object Detection,
// * including Java Builder structures for specifying Vision parameters.
// *
// * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
// */
//@TeleOp(name = "Visionportal4")
//public class Visionportal4 extends LinearOpMode {
//
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    private TfodProcessor tfod;
//    private VisionPortal visionPortal;
//
//
//    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
//    // this is only used for Android Studio when using models in Assets.
//    private static final String TFOD_MODEL_ASSET = "old1_model.tflite";
//    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
//    // this is used when uploading models directly to the RC using the model upload interface.
//    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
//    // Define the labels recognized in the model for TFOD (must be in training order!)
//    private static final String[] LABELS = {
//            "blueCrown",
//            "redCrown"
//    };
//
//
//    @Override
//    public void runOpMode() {
//
//        initTfod();
//        waitForStart();
//
//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//
//                telemetryTfod();
//
//                // Push telemetry to the Driver Station.
//                telemetry.update();
//
//                // Save CPU resources; can resume streaming when needed.
//                if (gamepad1.dpad_down) {
//                    visionPortal.stopStreaming();
//                } else if (gamepad1.dpad_up) {
//                    visionPortal.resumeStreaming();
//                }
//
//                // Share the CPU.
//                sleep(20);
//            }
//        }
//
//        // Save more CPU resources when camera is no longer needed.
//        visionPortal.close();
//
//    }   // end runOpMode()
//
//
//    private void initTfod() {
//
//        tfod = new TfodProcessor.Builder()
//                .setMaxNumRecognitions(10) // max # recognitions
//                .setUseObjectTracker(true) // use object tracker
//                .setTrackerMaxOverlap((float) 0.2) // max % of box overlapped by another box for recognition
//                .setTrackerMinSize(16) // minimum size of a tracked/recognized object (units?)
//
//
//                // With the following lines commented out, the default TfodProcessor Builder
//                // will load the default model for the season. To define a custom model to load,
//                // choose one of the following:
//                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
//                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                .setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                // The following default settings are available to un-comment and edit as needed to
//                // set parameters for custom models.
//                .setModelLabels(LABELS)
//                .setIsModelTensorFlow2(true)
//                .setIsModelQuantized(true)
//                .setModelInputSize(300)
//                .setModelAspectRatio(16.0 / 9.0)
//                .build();
//
//
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change name depending on mech team wants to name it
//                    .addProcessor(tfod) // add tfod processor
//                    .setCameraResolution(new Size(640, 480)) // import android.util.Size;
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // changed to MJPEG instead of YUY2 since it uses less bandwidth
//                    // .enableCameraMonitoring(true) // enableCameraMonitoring not identified? I also didn't find any related methods in VisionPortal.java
//                    .enableLiveView(true) // manually added this method because it was the closest thing to enableCameraMonitoring
//                    .setAutoStopLiveView(true)
//                    .build();
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessor(tfod) // add tfod processor
//                    .setCameraResolution(new Size(640, 480)) // import android.util.Size;
//                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // changed to MJPEG instead of YUY2 since it uses less bandwidth
//                    // .enableCameraMonitoring(true) // enableCameraMonitoring not identified? I also didn't find any related methods in VisionPortal.java
//                    .enableLiveView(true) // manually added this method because it was the closest thing to enableCameraMonitoring
//                    .setAutoStopLiveView(true)
//                    .build();
//        }
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        tfod.setMinResultConfidence(0.75f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        //visionPortal.setProcessorEnabled(tfod, true);
//
//    }
//
//
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }
//
//    }
//
//}
