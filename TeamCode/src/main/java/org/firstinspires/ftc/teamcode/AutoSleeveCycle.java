package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Color Sleeve Cycle")
public class AutoSleeveCycle extends LinearOpMode {
    // Name of the Webcam to be set in the config
    private final String webcamName = "Webcam 1";
    // Hardware
    private Hardware hardware;
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;

    static final int MOTOR_TICK_COUNTS = 1120;

    @Override
    public void runOpMode() throws InterruptedException {

        // Init hardware
        hardware = new Hardware(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();
        SleeveDetection.ParkingPosition position = sleeveDetection.getPosition();
        // the cycle

        // reset encoders
        hardware.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference = 3.14*4;
        double rotationsNeeded = 18/circumference;
        int encoderDrivingTarget = (int)(rotationsNeeded*1120);

        // set target
        hardware.frontRightMotor.setTargetPosition(encoderDrivingTarget);
        hardware.frontLeftMotor.setTargetPosition(encoderDrivingTarget);
        hardware.backRightMotor.setTargetPosition(encoderDrivingTarget);
        hardware.backLeftMotor.setTargetPosition(encoderDrivingTarget);

        // set speed
        hardware.frontRightMotor.setPower(0.5);
        hardware.frontLeftMotor.setPower(0.5);
        hardware.backRightMotor.setPower(0.5);
        hardware.backLeftMotor.setPower(0.5);

        // run
        hardware.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (hardware.frontRightMotor.isBusy() || hardware.frontLeftMotor.isBusy() || hardware.backRightMotor.isBusy() || hardware.backLeftMotor.isBusy()) {
            telemetry.addData("Path", "Moving");
            telemetry.update();
        }

        // stop motor
        hardware.frontRightMotor.setPower(0);
        hardware.frontLeftMotor.setPower(0);
        hardware.backRightMotor.setPower(0);
        hardware.backLeftMotor.setPower(0);

        // the parking after the cycle
        if (position == SleeveDetection.ParkingPosition.LEFT) {
            telemetry.addData("left", "4324");
        }

        else if (position == SleeveDetection.ParkingPosition.CENTER) {
            telemetry.addData("center", "4324");
        }

        else if (position == SleeveDetection.ParkingPosition.RIGHT) {
            telemetry.addData("right", "4324");
        }
    }
}