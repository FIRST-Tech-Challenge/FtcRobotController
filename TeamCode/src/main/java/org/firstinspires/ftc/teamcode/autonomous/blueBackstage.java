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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp
public class blueBackstage extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private static final String TFOD_MODEL_ASSET = "blueTeamPropIL.tflite";
    private static final String[] LABELS = {
            "BlueProp",
    };

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    String labelName;
    double middleOfImageWidth;
    double teamPropPosition;
    double stepNumber = 1;
    double leftSpeed = 0;
    double rightSpeed = 0;
    double driveTrainSpeed = 0;
    double correction;
    boolean timeToDefault = false;


    @Override
    public void runOpMode() {
        int spikeNumber = 0;
        initTfod();
        boolean detectionActive = true;
        spikeNumber = getSpikeNumber();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        DcMotor backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        Servo pixelPusher = hardwareMap.get(Servo.class,"pixelPusher");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class,"armMotor");
        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("spikeNumber",spikeNumber);
            telemetry.addData("ddet",detectionActive);
            telemetry.update();
            double driveTrainPoistion = backLeft.getCurrentPosition();
            if (detectionActive) {
                if (spikeNumber == 0) {
                    spikeNumber = getSpikeNumber();
                }
                else {
                    detectionActive = false;
                }

            }

            if (spikeNumber == 1) {
                blueLeft(robot,backLeft,imu,yawAngle,driveTrainPoistion, pixelPusher, armMotor);
            }
            if (spikeNumber == 2) {
                blueCenter(robot,backLeft,imu,yawAngle,driveTrainPoistion, pixelPusher, armMotor);
            }
            if (spikeNumber == 3) {
                blueRight(robot,backLeft,imu,yawAngle,driveTrainPoistion, pixelPusher, armMotor);
            }


        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()
    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(480)
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
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(tfod);


        visionPortal = builder.build();

        //tfod.setMinResultConfidence(50f);

    }   // end method initTfod()
    private int getSpikeNumber() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        int spikeNumber = 0;
        for (Recognition recognition : currentRecognitions) {
            middleOfImageWidth = (double) recognition.getImageWidth() / 2;
            teamPropPosition = (recognition.getLeft() + recognition.getRight()) / 2;
            telemetry.addData("Pixel", "Recognized");
            if (teamPropPosition < middleOfImageWidth) {
                spikeNumber = 1;
            } else if (teamPropPosition > middleOfImageWidth) {
                spikeNumber = 2;
            }
        }
        sleep(2500);
        if (currentRecognitions.isEmpty()) {
            sleep(2500);
            spikeNumber = 3;
        }

        return spikeNumber;
    }
    private void blueLeft (KDRobot robot, DcMotor backLeft, IMU imu, double yawAngle, double driveTrainPoistion, Servo pixelPusher, DcMotorEx armMotor){
        driveTrainPoistion = backLeft.getCurrentPosition();
        driveTrainSpeed = 0.27;
        yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (stepNumber == 1) {
            robot.setDriveTrainSpeed(0.4);
            robot.moveDriveTrain("STRAFE_LEFT");
            sleep(425);
            robot.stopDriveTrain();
            stepNumber = 2;
        }
        if (1125 >= driveTrainPoistion && stepNumber == 2) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 + correction;
            rightSpeed = -15 -correction;
            robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
            //robot.setDriveTrainSpeed(0.15);
            //robot.moveDriveTrain("FORWARD");
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 2){
            robot.stopDriveTrain();
            stepNumber = 3;
        }
        if (stepNumber == 3) {
            pixelPusher.setPosition(0.7);
            sleep(1000);
            stepNumber = 4;
        }
        if (stepNumber == 4) {
            robot.setWheelPower(-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed);
            sleep(500);
            robot.stopDriveTrain();
            stepNumber = 5;
        }
        if (stepNumber == 5) {
            driveTrainSpeed = 0.27;
            yawAngle = yawAngle * -1;
            stepNumber = 6;
        }
        if (stepNumber == 6) {
            if (yawAngle >= 89) {
                robot.stopDriveTrain();
                telemetry.addData("yawAngle > 90","STOP ACTION");
                telemetry.update();
                stepNumber = 7;
            } else if (yawAngle <= 90){
                robot.setWheelPower(driveTrainSpeed, -driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed);
                stepNumber = 5;
                telemetry.addData("keep rotating",yawAngle);
                telemetry.update();
            }
        }
        if (stepNumber == 7) {
            imu.resetYaw();
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveTrainPoistion = backLeft.getCurrentPosition();
            stepNumber = 8;
        }
        if (-750 <= driveTrainPoistion && stepNumber == 8) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 - correction;
            rightSpeed = -15 + correction;
            robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 8){
            robot.stopDriveTrain();
            stepNumber = 9;
        }

        if (stepNumber == 9) {
            robot.moveDriveTrain("STRAFE_LEFT");
            sleep (1000);
            stepNumber = 10;
        }
        if (stepNumber == 10) {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveTrainPoistion = backLeft.getCurrentPosition();
            stepNumber = 11;
        }
        if (-750 <= driveTrainPoistion && stepNumber == 11) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 - correction;
            rightSpeed = -15 + correction;
            robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 11){
            robot.stopDriveTrain();
            stepNumber = 12;
        }
        if (stepNumber == 12) {
            armMotor.setPower(0.35);
            sleep(1000);
            armMotor.setPower(0);
            stepNumber = 13;
        }


    }
    private void blueCenter (KDRobot robot, DcMotor backLeft, IMU imu, double yawAngle, double driveTrainPoistion, Servo pixelPusher, DcMotorEx armMotor){
        driveTrainPoistion = backLeft.getCurrentPosition();
        driveTrainSpeed = 0.27;
        yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (stepNumber == 1) {
            robot.setDriveTrainSpeed(0.4);
            robot.moveDriveTrain("STRAFE_RIGHT");
            sleep(100);
            robot.stopDriveTrain();
            stepNumber = 2;
        }
        if (1150 >= driveTrainPoistion && stepNumber == 2) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 + correction;
            rightSpeed = -15 -correction;
            robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
            //robot.setDriveTrainSpeed(0.15);
            //robot.moveDriveTrain("FORWARD");
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 2){
            robot.stopDriveTrain();
            stepNumber = 3;
        }
        if (stepNumber == 3) {
            pixelPusher.setPosition(0.7);
            sleep(1000);
            stepNumber = 4;
        }
        if (stepNumber == 4) {
            robot.setWheelPower(-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed,-driveTrainSpeed);
            sleep(500);
            robot.stopDriveTrain();
            stepNumber = 5;
        }
        if (stepNumber == 5) {
            yawAngle = yawAngle * -1;
            stepNumber  = 6;
        }
        if (stepNumber == 6) {
            if (yawAngle >= 89) {
                robot.stopDriveTrain();
                telemetry.addData("yawAngle > 90","STOP ACTION");
                telemetry.update();
                stepNumber = 7;
            } else if (yawAngle <= 90){
                robot.setWheelPower(driveTrainSpeed, -driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed);
                stepNumber = 5;
                telemetry.addData("keep rotating",yawAngle);
                telemetry.update();
            }
        }
        if (stepNumber == 7) {
            imu.resetYaw();
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveTrainPoistion = backLeft.getCurrentPosition();
            stepNumber = 8;
        }
        if (-875 <= driveTrainPoistion && stepNumber == 8) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 - correction;
            rightSpeed = -15 + correction;
            robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 8){
            robot.stopDriveTrain();
            stepNumber = 9;
        }
        if (stepNumber == 9) {
            robot.moveDriveTrain("STRAFE_LEFT");
            sleep (600);
            stepNumber = 10;
        }
        if (stepNumber == 10) {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveTrainPoistion = backLeft.getCurrentPosition();
            stepNumber = 11;
        }
        if (-1000 <= driveTrainPoistion && stepNumber == 11) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 - correction;
            rightSpeed = -15 + correction;
            robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 11){
            robot.stopDriveTrain();
            stepNumber = 12;
        }
        if (stepNumber == 12) {
            armMotor.setPower(0.35);
            sleep( 1000);
            armMotor.setPower(0);
            stepNumber = 13;
        }


    }
    private void blueRight (KDRobot robot, DcMotor backLeft, IMU imu, double yawAngle, double driveTrainPoistion, Servo pixelPusher, DcMotorEx armMotor){
        driveTrainPoistion = backLeft.getCurrentPosition();
        driveTrainSpeed = 0.27;
        yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (1225 >= driveTrainPoistion && stepNumber == 1) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 + correction;
            rightSpeed = -15 -correction;
            robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
            //robot.setDriveTrainSpeed(0.15);
            //robot.moveDriveTrain("FORWARD");
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 1){
            robot.stopDriveTrain();
            stepNumber = 2;
        }
        if (stepNumber == 2){
            yawAngle = yawAngle * -1;
            stepNumber = 3;
        }
        if (stepNumber == 3) {
            if (yawAngle >= 89) {
                robot.stopDriveTrain();
                telemetry.addData("yawAngle > 90","STOP ACTION");
                telemetry.update();
                stepNumber = 4;
            } else if (yawAngle <= 90){
                robot.setWheelPower(0.15, -0.15, 0.15, -0.15);
                telemetry.addData("keep rotating",yawAngle);
                telemetry.update();
                stepNumber = 2;
            }
        }
        if (stepNumber == 4) {
            pixelPusher.setPosition(0.7);
            sleep(500);
            stepNumber = 5;
        }
        if (stepNumber == 5) {
            imu.resetYaw();
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveTrainPoistion = backLeft.getCurrentPosition();
            stepNumber = 6;
        }
        if (-1750 <= driveTrainPoistion && stepNumber == 6) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -15 - correction;
            rightSpeed = -15 + correction;
            robot.setWheelPower(leftSpeed/100,rightSpeed/100,leftSpeed/100,rightSpeed/100);
            telemetry.addData("driveTrainPosition",driveTrainPoistion);
            telemetry.update();
        }
        else if (stepNumber == 6){
            robot.stopDriveTrain();
            stepNumber = 7;
        }
        if (stepNumber == 7) {
            armMotor.setPower(0.35);
            sleep(1000);
            armMotor.setPower(0);
            stepNumber = 8;
        }

    }

}