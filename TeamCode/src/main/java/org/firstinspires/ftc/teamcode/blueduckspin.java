/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Constants.SamplingLocation;
import com.qualcomm.robotcore.hardware.DcMotor;


import java.util.List;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 * hi
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="boringggg", group="Pushbot")
public class blueduckspin extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private ElapsedTime runtime = new ElapsedTime();
    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel",
    };



    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        // initVuforia();
//        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
//        if (tfod != null) {
//            tfod.activate();

//             // The TensorFlow software will scale the input images from the camera to a lower resolution.
//             // This can result in lower detection accuracy at longer distances (> 55cm or 22").
//             // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
//             // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
//             // should be set to the value of the images used to create the TensorFlow Object Detection model
//             // (typically 16/9).
//            tfod.setZoom(1.0, 16 / 9.0);
//         }


        /* Declare OpMode members. */
        HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
        robot.init(hardwareMap);
        ElapsedTime     runtime = new ElapsedTime();


        final double     FORWARD_SPEED = 0.3;
        final double     TURN_SPEED    = 0.3;
        int markerPosition = 3;
        int frontRightPosition = 0;
        int frontLeftPosition = 0;
        int backRightPosition = 0;
        int backLeftPosition = 0;

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        waitForStart();


        /*frontLeftPosition += 1000;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontLeft.setPower(0.5);
        robot.frontRight.setPower(0.5);
        robot.backRight.setPower(0.5);
        robot.backLeft.setPower(0.5);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        frontLeftPosition -= 1000;
        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        frontRightPosition += 1000;
        robot.frontRight.setTargetPosition(frontLeftPosition);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        frontRightPosition -= 1000;
        robot.frontRight.setTargetPosition(frontLeftPosition);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        //backLeftPosition += 1000;
        //robot.backleft.setTargetPosition(frontLeftPositon;
        //robot.backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        backLeftPosition -= 1000;
        robot.backLeft.setTargetPosition(frontLeftPosition);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        backRightPosition += 1000;
        robot.backRight.setTargetPosition(frontLeftPosition);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        backRightPosition -= 1000;
        robot.backRight.setTargetPosition(frontLeftPosition);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);



        /*robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.claw.setPosition(0.3);
        sleep(1000);
        robot.claw.setPosition(-1);
        sleep(1000);

        robot.liftLeft.setPower(0.5);
        robot.liftRight.setPower(0.5);
        sleep(250);
        robot.liftLeft.setPower(0);
        robot.liftRight.setPower(0);

        frontLeftPosition += 900;
        frontRightPosition += 900;
        backLeftPosition -= 900;
        backRightPosition -= 900;
        //going forward

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);

        robot.frontLeft.setPower(0.3);
        robot.frontRight.setPower(0.3);
        robot.backRight.setPower(0.3);
        robot.backLeft.setPower(0.3);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);

        robot.claw.setPosition(1);
        sleep(2000);


        frontLeftPosition -= 200;
        frontRightPosition -= 200;
        backLeftPosition += 200;
        backRightPosition += 200;
        //going right sideways

        robot.frontLeft.setTargetPosition(frontLeftPosition);
        robot.frontRight.setTargetPosition(frontRightPosition);
        robot.backLeft.setTargetPosition(backLeftPosition);
        robot.backRight.setTargetPosition(backRightPosition);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);*/









    }



    }
