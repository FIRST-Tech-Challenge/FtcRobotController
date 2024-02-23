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
@Disabled
public class PlaceLinePixel extends LinearOpMode {

    public static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    public static final String TFOD_MODEL_ASSET = "Firewall_Centerstage_AI_v4.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    public static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    public static final String[] LABELS = {
            "Custom Element Blue", "Custom Element Red",
    };

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor armExt;
    public DcMotor armBrace;
    public DcMotor armRotate;
    public Servo servoLeft;
    public Servo servoRight;

    public double index = 0;

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
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

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
            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(200);

            RobotTurnLeft();
            TimeUnit.MILLISECONDS.sleep(80);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(60);

            RobotMoveBackwardHalf();
            TimeUnit.MILLISECONDS.sleep(60);

            RobotTurnRight();
            TimeUnit.MILLISECONDS.sleep(90);

            RobotMoveBackwardHalf();
            TimeUnit.MILLISECONDS.sleep(90);

            RobotStop();
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void PixelLocation2() {
        try {
           RobotMoveFarwardHalf();
           TimeUnit.MILLISECONDS.sleep(500);

           RobotMoveBackwardHalf();

           frontRightMotor.setPower(-.6);
           TimeUnit.MILLISECONDS.sleep(40);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void PixelLocation3() {
        try {
            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(200);

            RobotTurnRight();
            TimeUnit.MILLISECONDS.sleep(150);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(170);

            RobotMoveBackwardHalf();
            TimeUnit.MILLISECONDS.sleep(170);

            RobotTurnLeft();
            TimeUnit.MILLISECONDS.sleep(160);

            RobotMoveBackwardHalf();
            TimeUnit.MILLISECONDS.sleep(110);

            RobotStop();
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RedLocation1() {
        try {
            RobotTurnRight();
            TimeUnit.MILLISECONDS.sleep(650);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotStrafeLeftHalf();
            TimeUnit.MILLISECONDS.sleep(800);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RedLocation2() {
        try {
            RobotTurnRight();
            TimeUnit.MILLISECONDS.sleep(1530);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);

            armBrace.setPower(-1);
            armBrace.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(200);
            armBrace.setPower(0);
            armRotate.setPower(0);

            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(1);
            backRightMotor.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(3200);

            RobotStop();
            TimeUnit.MILLISECONDS.sleep(100);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BlueLocation1() {
        try {
            RobotTurnLeft();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotStrafeRightHalf();
            TimeUnit.MILLISECONDS.sleep(800);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BlueLocation2() {
        try {
            RobotTurnLeft();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotMoveFarwardHalf();

            armBrace.setPower(-.2);
            armRotate.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(500);

            armRotate.setPower(0);
            armBrace.setPower(0);
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarwardHalf();

            armUp();

            RobotStrafeRightHalf();
            TimeUnit.MILLISECONDS.sleep(800);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BlueLocation1Spike() {
        try {
            RobotTurnLeft();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(200);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BlueLocation2Spike() {
        try {
            RobotTurnLeft();
            TimeUnit.MILLISECONDS.sleep(500);

            RobotMoveFarwardHalf();

            armBrace.setPower(-.2);
            armRotate.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(500);

            armRotate.setPower(0);
            armBrace.setPower(0);
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarwardHalf();
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RedLocation1Spike() {
        try {
            RobotTurnRight();
            TimeUnit.MILLISECONDS.sleep(650);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(200);;
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RedLocation2Spike() {
        try {

            TimeUnit.MILLISECONDS.sleep(100);

        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BoardPixel1() {
        try {
            RobotStrafeLeftHalf();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStop();

            armExt.setPower(1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);

            armBrace.setPower(-.2);
            armRotate.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(500);

            armBrace.setPower(0);
            armRotate.setPower(0);

            TimeUnit.MILLISECONDS.sleep(500);

            Release();

            TimeUnit.MILLISECONDS.sleep(300);

            armBrace.setPower(.3);
            armRotate.setPower(.3);
            TimeUnit.MILLISECONDS.sleep(500);

            armBrace.setPower(0);
            armRotate.setPower(0);
            armExt.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(400);

            armExt.setPower(0);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BoardPixel2() {
        try {
            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStop();

            armExt.setPower(1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);

            armBrace.setPower(-.2);
            armRotate.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(500);

            armBrace.setPower(0);
            armRotate.setPower(0);

            TimeUnit.MILLISECONDS.sleep(500);

            Release();

            TimeUnit.MILLISECONDS.sleep(300);

            armBrace.setPower(.3);
            armRotate.setPower(.3);
            TimeUnit.MILLISECONDS.sleep(500);

            armBrace.setPower(0);
            armRotate.setPower(0);
            armExt.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(400);

            armExt.setPower(0);
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void BoardPixel3() {
        try {
            RobotStrafeRightHalf();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotMoveFarwardHalf();
            TimeUnit.MILLISECONDS.sleep(100);

            RobotStop();

            armExt.setPower(1);
            TimeUnit.MILLISECONDS.sleep(500);

            armExt.setPower(0);

            armBrace.setPower(-.2);
            armRotate.setPower(-.2);
            TimeUnit.MILLISECONDS.sleep(500);

            armBrace.setPower(0);
            armRotate.setPower(0);

            TimeUnit.MILLISECONDS.sleep(500);

            Release();

            TimeUnit.MILLISECONDS.sleep(300);

            armBrace.setPower(.3);
            armRotate.setPower(.3);
            TimeUnit.MILLISECONDS.sleep(500);

            armBrace.setPower(0);
            armRotate.setPower(0);
            armExt.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(400);

            armExt.setPower(0);
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
        index = 0;
        try {
            while (index <= 1) {
                frontLeftMotor.setPower(-index);
                frontRightMotor.setPower(index);
                backLeftMotor.setPower(-index);
                backRightMotor.setPower(index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotMoveFarwardHalf() {
        index = 0;
        try {
            while (index <= 0.5) {
                frontLeftMotor.setPower(-index);
                frontRightMotor.setPower(index);
                backLeftMotor.setPower(-index);
                backRightMotor.setPower(index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotMoveBackward() {
        index = 0;
        try {
            while (index <= 1) {
                frontLeftMotor.setPower(index);
                frontRightMotor.setPower(-index);
                backLeftMotor.setPower(index);
                backRightMotor.setPower(-index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotMoveBackwardHalf() {
        index = 0;
        try {
            while (index <= 0.5) {
                frontLeftMotor.setPower(index);
                frontRightMotor.setPower(-index);
                backLeftMotor.setPower(index);
                backRightMotor.setPower(-index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotTurnRight() {
        index = 0;
        try {
            while (index <= 0.5) {
                frontLeftMotor.setPower(-index);
                frontRightMotor.setPower(-index);
                backLeftMotor.setPower(-index);
                backRightMotor.setPower(-index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotTurnLeft() {
        index = 0;
        try {
            while (index <= 0.5) {
                frontLeftMotor.setPower(index);
                frontRightMotor.setPower(index);
                backLeftMotor.setPower(index);
                backRightMotor.setPower(index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotStop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void RobotStrafeLeft() {
        index = 0;
        try {
            while (index <= 1) {
                frontLeftMotor.setPower(-index);
                frontRightMotor.setPower(index);
                backLeftMotor.setPower(index);
                backRightMotor.setPower(-index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotStrafeLeftHalf() {
        index = 0;
        try {
            while (index <= 0.5) {
                frontLeftMotor.setPower(-index);
                frontRightMotor.setPower(index);
                backLeftMotor.setPower(index);
                backRightMotor.setPower(-index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotStrafeRight() {
        index = 0;
        try {
            while (index <= 1) {
                frontLeftMotor.setPower(index);
                frontRightMotor.setPower(-index);
                backLeftMotor.setPower(-index);
                backRightMotor.setPower(index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void RobotStrafeRightHalf() {
        index = 0;
        try {
            while (index <= 0.5) {
                frontLeftMotor.setPower(index);
                frontRightMotor.setPower(-index);
                backLeftMotor.setPower(-index);
                backRightMotor.setPower(index);
                telemetry.addData("Index: ", index);
                telemetry.update();
                index += 0.1;
                TimeUnit.MILLISECONDS.sleep(50);
            }
        } catch (InterruptedException e) {
            //Nothing
        }
    }

    public void Grab() {
        servoLeft.setPosition(0.3);
        servoRight.setPosition(0.7);
    }

    public void Release() {
        servoLeft.setPosition(0);
        servoRight.setPosition(1);
    }

    public void Break() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Float() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}   // end class
