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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

@Autonomous(name = "TFOD Easy", group = "Concept")

public class bluerightfirst extends LinearOpMode {
    public DcMotor back_Left;
    public DcMotor back_Right;
    public Blinker expansion_Hub_2;
    public Blinker expansion_Hub_9;
    public DcMotor front_Left;
    public DcMotor front_Right;
    public Gyroscope imu_1;
    public Gyroscope imu;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    //The variable to store our instance of the TensorFlow Object Detection processor.

    private TfodProcessor tfod;
    //The variable to store our instance of the vision portal.

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

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

    //Initialize the TensorFlow Object Detection processor.

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

    //Add telemetry about TensorFlow Object Detection (TFOD) recognitions.

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            int position;

            if (recognition = 1);
            {

                if (x < 200) {
                    position = 1;
                else {
                    position = 2;
                    }
                }

                if (position == 1) ;
                //go forward
                back_Left.setTargetPosition(700);
                back_Right.setTargetPosition(700);
                front_Left.setTargetPosition(700);
                front_Right.setTargetPosition(700);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                    telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                    telemetry.update();
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (position == 2) ;
                //go forward
                back_Left.setTargetPosition(700);
                back_Right.setTargetPosition(700);
                front_Left.setTargetPosition(700);
                front_Right.setTargetPosition(700);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                    telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                    telemetry.update();
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                if (position == 3) ;
                //go forward
                back_Left.setTargetPosition(700);
                back_Right.setTargetPosition(700);
                front_Left.setTargetPosition(700);
                front_Right.setTargetPosition(700);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                    telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                    telemetry.update();
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }

        }

    }
}
