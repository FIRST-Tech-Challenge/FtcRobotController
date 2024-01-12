/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.utils;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Place Line Pixel")
public class PlaceLinePixel extends LinearOpMode {

    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    public static final String TFOD_MODEL_ASSET = "Firewall_Centerstage_AI_v3.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    public static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    public static final String[] LABELS = {
            "Custom Element",
    };

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor armExt;
    public DcMotor armBrace;
    public DcMotor armRotate;
    public Servo linearGripper;

    public boolean Location1 = false;
    public boolean Location2 = false;
    public boolean Location3 = false;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    public TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        linearGripper = hardwareMap.get(Servo.class, "linearGripper");

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                RobotMoveFarward();

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

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
            if (x < 428) {
                telemetry.addData("Position 1", x);
                Location1 = true;
            } else if (x > 428 && x < 856) {
                telemetry.addData("Position 2", x);
                Location2 = true;
            } else if (x > 856) {
                telemetry.addData("Position 3", x);
                Location3 = true;
            }

        }   // end for() loop

    }   // end method telemetryTfod()

    public void PixelLocation1() {
        try {
            RobotMoveFarward();
            TimeUnit.MILLISECONDS.sleep(600);

            frontLeftMotor.setPower(.3);
            backLeftMotor.setPower(.3);
            TimeUnit.MILLISECONDS.sleep(500);

            frontLeftMotor.setPower(0);
            backRightMotor.setPower(-.3);
            TimeUnit.MILLISECONDS.sleep(1700);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(200);

            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(500);

            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            TimeUnit.MILLISECONDS.sleep(1250);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(600);

            RobotStop();
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void PixelLocation2() {
        try {
            RobotStrafeRight();
            TimeUnit.MILLISECONDS.sleep(950);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarward();
            TimeUnit.MILLISECONDS.sleep(2000);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(200);

            RobotStrafeLeft();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(200);

            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStrafeLeft();
            TimeUnit.MILLISECONDS.sleep(300);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(900);



        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void PixelLocation3() {
        try {
            RobotMoveFarward();
            TimeUnit.MILLISECONDS.sleep(600);

            frontRightMotor.setPower(-.3);
            backRightMotor.setPower(-.3);
            TimeUnit.MILLISECONDS.sleep(500);

            frontRightMotor.setPower(-0);
            backLeftMotor.setPower(.3);
            TimeUnit.MILLISECONDS.sleep(1700);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(200);

            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(500);

            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            TimeUnit.MILLISECONDS.sleep(1100);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(750);

            RobotStop();
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RedLocation1() {
        //Movement Code
    }

    public void RedLocation2() {
        try {
            RobotTurnRight();
            TimeUnit.MILLISECONDS.sleep(1450);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarward();
            TimeUnit.MILLISECONDS.sleep(2000);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStrafeLeft();
            TimeUnit.MILLISECONDS.sleep(2800);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BlueLocation1() {
        //Movement Code
    }

    public void BlueLocation2() {
        //MovementCode
    }

    public void BoardPixel1() {
        try {
            RobotStrafeLeft();
            TimeUnit.MILLISECONDS.sleep(700);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarward();
            TimeUnit.MILLISECONDS.sleep(1300);

            RobotStop();

            armExt.setPower(1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);

            armRotate.setPower(-.2);
            armBrace.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(1000);

            armBrace.setPower(.05);
            armRotate.setPower(.05);
            TimeUnit.MILLISECONDS.sleep(1000);

            linearGripper.setPosition(.3);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);
            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStop();
            armRotate.setPower(-.2);
            armBrace.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BoardPixel2() {
        try {
            RobotMoveFarward();
            TimeUnit.MILLISECONDS.sleep(1200);

            RobotStop();

            armExt.setPower(1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);

            armRotate.setPower(-.2);
            armBrace.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(1000);

            armBrace.setPower(.05);
            armRotate.setPower(.05);
            TimeUnit.MILLISECONDS.sleep(1000);

            linearGripper.setPosition(.3);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);
            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStop();
            armRotate.setPower(-.2);
            armBrace.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BoardPixel3() {
        try {
            RobotStrafeRight();
            TimeUnit.MILLISECONDS.sleep(1000);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarward();
            TimeUnit.MILLISECONDS.sleep(1300);

            RobotStop();

            armExt.setPower(1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);

            armRotate.setPower(-.2);
            armBrace.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(1000);

            armBrace.setPower(.05);
            armRotate.setPower(.05);
            TimeUnit.MILLISECONDS.sleep(1000);

            linearGripper.setPosition(.3);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);
            RobotMoveBackward();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStop();
            armRotate.setPower(-.2);
            armBrace.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(300);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void armUp() {
        try {
            armRotate.setPower(.3);
            armBrace.setPower(.3);
            TimeUnit.MILLISECONDS.sleep(1600);
            armRotate.setPower(0);
            armBrace.setPower(0);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotMoveFarward() {
        frontLeftMotor.setPower(.5);
        frontRightMotor.setPower(-.5);
        backLeftMotor.setPower(.5);
        backRightMotor.setPower(-.5);
    }

    public void RobotMoveBackward() {
        frontLeftMotor.setPower(-.5);
        frontRightMotor.setPower(.5);
        backLeftMotor.setPower(-.5);
        backRightMotor.setPower(.5);
    }

    public void RobotTurnRight() {
        frontLeftMotor.setPower(.5);
        frontRightMotor.setPower(.5);
        backLeftMotor.setPower(.5);
        backRightMotor.setPower(.5);
    }

    public void RobotTurnLeft() {
        frontLeftMotor.setPower(-.5);
        frontRightMotor.setPower(-5);
        backLeftMotor.setPower(-.5);
        backRightMotor.setPower(-.5);
    }

    public void RobotStop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void RobotStrafeLeft() {
        frontLeftMotor.setPower(-.5);
        frontRightMotor.setPower(-.5);
        backLeftMotor.setPower(.45);
        backRightMotor.setPower(.5);
    }

    public void RobotStrafeRight() {
        frontLeftMotor.setPower(.5);
        frontRightMotor.setPower(.5);
        backLeftMotor.setPower(-.45);
        backRightMotor.setPower(-.5);
    }

}   // end class
