package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Auto", group="Auto")
@Disabled
public class Auto extends LinearOpMode {

    //Declare motors/servos variables
    private ElapsedTime runtime = new ElapsedTime();
    //Set Motor objects
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = true; // change to true if using webcam
    private static final String WEBCAM_NAME = "Webcam"; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {

        //Define motors/servos hardware maps
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        //Set Run modes
        leftFront.setRunMode(Motor.RunMode.RawPower);
        rightFront.setRunMode(Motor.RunMode.RawPower);
        leftBack.setRunMode(Motor.RunMode.RawPower);
        rightBack.setRunMode(Motor.RunMode.RawPower);

        //Set Directions
        leftFront.setInverted(true);
        rightFront.setInverted(false);
        leftBack.setInverted(true);
        rightBack.setInverted(false);

//        int cameraMonitorViewId = this
//                .hardwareMap
//                .appContext
//                .getResources().getIdentifier(
//                        "cameraMonitorViewId",
//                        "id",
//                        hardwareMap.appContext.getPackageName()
//                );
////        if (USING_WEBCAM) {
//        camera = OpenCvCameraFactory
//                .getInstance()
//                .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
////        } else {
////            camera = OpenCvCameraFactory
////                    .getInstance()
////                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
////        }
//
//        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));
//
//        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);
//
//        UGContourRingPipeline.Config.setHORIZON(HORIZON);
//
//        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        //Initialized
        telemetry.addData("Status", "Initialized");

//        String height = String.valueOf(pipeline.getHeight());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        switch (height) {
//            case "ZERO":
//                telemetry.addData("Path", "Running Path 0");
//                break;
//            case "ONE":
//                telemetry.addData("Path", "Running Path 1");
//                break;
//            case "FOUR":
//                telemetry.addData("Path", "Running Path 4");
//                break;
//            default:
//        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }



}

