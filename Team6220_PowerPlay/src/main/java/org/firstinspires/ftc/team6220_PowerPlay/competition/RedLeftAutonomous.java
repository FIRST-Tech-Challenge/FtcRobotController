package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetectionPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.GrabberCameraPipeline;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
@Autonomous(name = "RedLeftAutonomous")
public class RedLeftAutonomous extends BaseAutonomous {

    public AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public RobotCameraPipeline robotCameraPipeline;
    public GrabberCameraPipeline grabberCameraPipeline;

    public OpenCvCamera robotCamera;
    public OpenCvCamera grabberCamera;

    public int cameraMonitorViewId;

    int signal;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
//        grabberCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "GrabberCamera"), cameraMonitorViewId);

//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();
        robotCameraPipeline = new RobotCameraPipeline(Constants.LOWER_RED, Constants.UPPER_RED);
//        grabberCameraPipeline = new GrabberCameraPipeline();

        robotCamera.setPipeline(robotCameraPipeline);

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

//        grabberCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                grabberCamera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {}
//        });

        //signal = detectSignal();

        waitForStart();

        //robotCamera.setPipeline(robotCameraPipeline);
        //grabberCamera.setPipeline(grabberCameraPipeline);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(1000);

        driveSlidesAutonomous(Constants.SLIDE_STOW);

        driveInches(7.32, 55);

        turnToAngle(-45);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        turnToAngle(90);

        driveSlides(Constants.SLIDE_STACK_FOUR);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlides(Constants.SLIDE_LOW);

        driveInches(153.4, 6.7);

        turnToAngle(180);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        turnToAngle(90);

        driveSlides(Constants.SLIDE_STACK_THREE);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlides(Constants.SLIDE_LOW);

        driveInches(153.4, 6.7);

        turnToAngle(180);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        turnToAngle(90);

        driveSlides(Constants.SLIDE_STACK_TWO);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlides(Constants.SLIDE_LOW);

        driveInches(153.4, 6.7);

        turnToAngle(180);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        turnToAngle(90);

        driveSlides(Constants.SLIDE_STACK_ONE);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlides(Constants.SLIDE_LOW);

        driveInches(153.4, 6.7);

        turnToAngle(180);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        sleep(500);

        turnToAngle(90);

        driveSlides(Constants.SLIDE_BOTTOM);

        centerConeStack(robotCameraPipeline);

        sleep(500);

        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        sleep(500);

        driveSlides(Constants.SLIDE_LOW);

        driveInches(153.4, 6.7);

        turnToAngle(180);

        sleep(500);

        driveGrabber(Constants.GRABBER_OPEN_POSITION);
    }
}
