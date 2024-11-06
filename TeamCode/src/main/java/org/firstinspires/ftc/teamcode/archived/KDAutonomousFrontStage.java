package org.firstinspires.ftc.teamcode.archived;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Disabled
public class KDAutonomousFrontStage extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    boolean is_pixel_detected = false;
    int stepNumber = 0;
    double drivetrainspeed = 0.3;
    boolean armRaised = false;
    boolean yawAngleUpdated = false;
    boolean yawAngleReset = false;
    boolean driveTrainMovedForward = false;
    boolean robotTurnedToSpikeMark = false;
    boolean robotOutake = false;
    boolean robotTurnedToSpike2 = false;

    @Override
    public void runOpMode() {
        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        DcMotor andymark = hardwareMap.get(DcMotor.class,"andymark");
        DcMotor armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);

        IMU.Parameters myIMUparameters;
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();

        initTfod();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }
            // Share the CPUªª.
            sleep(20);

            int spike_number = this.detectSpikeMarkOfPixel(cameraServo);
            // spike_number = 2; // for testing
            this.moveRobotToSpikeMark(spike_number, robot, imu, andymark, armMotor);
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    // this method returns the spike mark number where the pixel is located.
    private int detectSpikeMarkOfPixel(Servo cameraServo) {
        int pixel_detected_at = 0;
        int check_pixel_at_position = 1;
        Recognition pixel = null;

        if (!is_pixel_detected && check_pixel_at_position == 1) {
            cameraServo.setPosition(0.8);
            sleep(30);
            // telemetry.addData("check_pixel_at_position ", check_pixel_at_position);
            // telemetry.addData("is_pixel_detected ", is_pixel_detected);
            // telemetry.update();
            sleep(3000);
            pixel = detectPixels();
            if (pixel != null) {
                pixel_detected_at = 1;
                is_pixel_detected = true;
                telemetry.addData("Pixel detected at ", pixel_detected_at);
                telemetry.addData("Servo Position ",cameraServo.getPosition());
            } else {
                telemetry.addData("No Objects Detected "," at position 1");
            }
            telemetry.update();
            sleep(2000);
            check_pixel_at_position = 3;
        }
        if (!is_pixel_detected && check_pixel_at_position == 3) {
            cameraServo.setPosition(1.3);
            sleep(30);
            telemetry.addData("check_pixel_at_position ", check_pixel_at_position);
            telemetry.addData("is_pixel_detected ", is_pixel_detected);
            telemetry.update();
            sleep(3000);
            pixel =  null;
            pixel = detectPixels();
            if (pixel != null) {
                pixel_detected_at = 3;
                is_pixel_detected = true;
                telemetry.addData("Pixel detected at ", pixel_detected_at);
                telemetry.addData("Servo Position ",cameraServo.getPosition());
            }  else {
                telemetry.addData("No Objects Detected ","at position 3");
            }
            telemetry.update();
            sleep(2000);
        }
        if (!is_pixel_detected) {
            pixel_detected_at = 2;
            is_pixel_detected = true;
            telemetry.addData("Pixel detected at ", pixel_detected_at);
            telemetry.addData("Servo Position ", cameraServo.getPosition());
            telemetry.update();
            sleep(2000);
        }
        return pixel_detected_at;
    }

    private void moveRobotToSpikeMark(int spike_number, KDRobot robot, IMU imu, DcMotor andymark, DcMotor armMotor){
        double yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        yawAngleUpdated = false;
        if (!armRaised) {
            armMotor.setPower(0.35);
            sleep(500);
            armMotor.setPower(0.1);
            armRaised = true;
        }
        if (!driveTrainMovedForward) {
            robot.setDriveTrainSpeed(0.4);
            robot.moveDriveTrain(DriveTrainDirection.FORWARD);
            sleep (1050);
            robot.stopDriveTrain();
            driveTrainMovedForward = true;
            sleep(250);
            stepNumber = 2;
        }
        if (stepNumber == 2 && driveTrainMovedForward) {
            drivetrainspeed = 0.3;
            yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            yawAngle = Math.round(yawAngle);
            if (spike_number == 1) {
                yawAngle = yawAngle * 1;
                drivetrainspeed = drivetrainspeed * 1;
                yawAngleUpdated = true;
            }
            if (spike_number == 2) {
                stepNumber = 3;
            }
            if (spike_number == 3){
                yawAngle = yawAngle * -1;
                drivetrainspeed = drivetrainspeed * -1;
                yawAngleUpdated = true;
            }
            if (yawAngleUpdated) {
                if (yawAngle >= 90) {
                    robot.stopDriveTrain();
                    stepNumber = 3; // Stop Rotating
                    telemetry.addData("yawAngle > 90","STOP ACTION");
                    telemetry.update();
                } else if (yawAngle <= 85){
                    robot.setWheelPower(-drivetrainspeed, drivetrainspeed, -drivetrainspeed, drivetrainspeed);
                    telemetry.addData("SpikeNumber",spike_number);
                    telemetry.addData("keep rotating",yawAngle);
                    telemetry.update();
                }
            }
        }
        if (stepNumber == 3) {
            armMotor.setPower(0);
            sleep(200);
            armMotor.setPower(0.1);
            andymark.setPower(-0.175);
            sleep(2000);
            andymark.setPower(0);
            armMotor.setPower(0.35);
            sleep(200);
            armMotor.setPower(0);
            sleep(650);
            andymark.setPower(-0.2);
            sleep(2000);
            andymark.setPower(0);
            stepNumber = 4;
        }
    }

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
    private Recognition detectPixels() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        //telemetry.addData("# Objects Detected", currentRecognitions.size());

        Recognition recognizedPixel = null;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (recognition.getConfidence() * 100 >= 80) {
                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.update();
                return recognition;
            }
            break;
        }   // end for() loop
        return null;
    }   // end method telemetryTfod()
}