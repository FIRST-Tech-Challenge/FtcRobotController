package org.firstinspires.ftc.Team19567.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.pipeline.tsePipeline;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "FSM", group = "Dababy")
public class redWarehouseFSM extends LinearOpMode {

    private tsePipeline pipeline = new tsePipeline(telemetry);
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    private Mechanisms mechanisms;

    public enum State {
        TRAJECTORY_1,
        TRAJECTORY_2,
        TURN_1,
        TRAJECTORY_3,
        WAIT_1,
        TURN_2,
        IDLE
    }

    AUTO_STATE currentState = AUTO_STATE.DETECTING_OPENCV;

    Pose2d startPose = new Pose2d(10, -63, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);
        mechanisms = new Mechanisms(hardwareMap,telemetry);
        mechanisms.setModes();

        chassis.setPoseEstimate(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(544,288, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("OpenCV","OpenCV actually connected wow");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV","OpenCV failed to load :( Error Code: " + errorCode);
                telemetry.update();
            }
        });

        while(!opModeIsActive()) {
            location = pipeline.getLocation();
            telemetry.addData("location",location);
            telemetry.update();
        }
        while(!isStopRequested()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        currentState = AUTO_STATE.MOVING_TO_HUB;

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case MOVING_TO_HUB: {
                    if (!chassis.isBusy()) {
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                    }
                    break;
                }
                case INTAKING_FREIGHT: {
                    if (!chassis.isBusy()) {
                        currentState = AUTO_STATE.MOVING_TO_HUB;
                    }
                    break;
                }
                default: {
                    if(!chassis.isBusy()) {
                        currentState = AUTO_STATE.MOVING_TO_HUB;
                    }
                }
            }

            chassis.update();

            mechanisms.maintainBalance();

            Pose2d poseEstimate = chassis.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("Heading", poseEstimate.getHeading());
            telemetry.addData("State",currentState);
            telemetry.update();
        }
    }
}