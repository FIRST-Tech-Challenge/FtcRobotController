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

package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainDirection;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class KDAutonomousILFrontStage extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    private static final String TFOD_MODEL_ASSET = "RedTeamProp.tflite";
    private static final String[] LABELS = {
            "RedProp",
    };

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    double middleOfImageWidth;
    double teamPropPosition;
    boolean outake = false;
    boolean robotMovedForward = false;

    @Override
    public void runOpMode() {

        initTfod();
        sleep(5000);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        int spikeNumber = 0;
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        Servo pixelPusher = hardwareMap.get(Servo.class,"pixelPusher");
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spikeNumber = getSpikeNumber();

        //REMOVE NEXT LINE FOR TESTING ONLY
        spikeNumber = 3;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                moveRobotToSpikeMark(robot, spikeNumber, imu, backLeft);
                if (outake) {
                    pixelPusher.setPosition(1);
                }
            }
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(50);
    }   // end method initTfod()


    private int getSpikeNumber() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        int spikeNumber = 0;
        if (currentRecognitions.isEmpty()) {
            spikeNumber = 3;
        }
        for (Recognition recognition : currentRecognitions) {
            middleOfImageWidth = (double) recognition.getImageWidth() / 2;
            teamPropPosition = (recognition.getLeft() + recognition.getRight()) / 2;
            telemetry.addData("Pixel", "Recognized");
            telemetry.addData("Conidence",recognition.getConfidence());
            if (teamPropPosition < middleOfImageWidth) {
                spikeNumber = 1;
            } else if (teamPropPosition > middleOfImageWidth) {
                spikeNumber = 2;
            }
        }
        return spikeNumber;
    }

    private void moveRobotToSpikeMark(KDRobot robot, int spikeNumber, IMU imu, DcMotor backLeft) {

        boolean moveForward = false;
        double correction;
        double leftSpeed;
        double rightSpeed;
        double targetY = 1117;
        double turnDirection = 0;
        double driveTrainSpeed = 0;
        boolean yawAngleUpdated = false;

        double YawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double driveTrainPoistion = backLeft.getCurrentPosition();
        YawAngle = (double) Math.round(YawAngle * 10) / 10.0;
        driveTrainSpeed = 0.27;

        //telemetry.addData("yawAngle",YawAngle);
        //telemetry.addData("pos",driveTrainPosition);
        telemetry.update();
        if (targetY >= driveTrainPoistion && !robotMovedForward) {
            correction = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            leftSpeed = -25 + correction;
            rightSpeed = -25 -correction;
            robot.setWheelPower(leftSpeed/-100,rightSpeed/-100,leftSpeed/-100,rightSpeed/-100);
        }
        else {
            robot.stopDriveTrain();
            robotMovedForward = true;
        }
        if (spikeNumber == 1) {
            turnDirection = -1;
        } else if (spikeNumber == 3) {
            turnDirection = 1;
        }
        if (turnDirection == -1) {
            yawAngleUpdated = true;
        }
        if (turnDirection == 1) {
            YawAngle = YawAngle * -1;
            driveTrainSpeed = driveTrainSpeed * -1;
            yawAngleUpdated = true;
        }
        if (yawAngleUpdated && robotMovedForward) {
            if (YawAngle >= 89) {
                robot.stopDriveTrain();
                //telemetry.addData("yawAngle > 90","STOP ACTION");
                telemetry.update();
                moveForward = true;
            } else if (YawAngle <= 90){
                robot.setWheelPower(-driveTrainSpeed, driveTrainSpeed, -driveTrainSpeed, driveTrainSpeed);
                //telemetry.addData("keep rotating",YawAngle);
                telemetry.update();
            }
        }
        if (moveForward && !outake) {
            robot.setDriveTrainSpeed(0.3);
            robot.moveDriveTrain(DriveTrainDirection.FORWARD);
            sleep(500);
            robot.stopDriveTrain();
            outake = true;
        }
    }
}