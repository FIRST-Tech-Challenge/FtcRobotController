package org.firstinspires.ftc.Team19567.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.pipeline.COLOR;
import org.firstinspires.ftc.Team19567.pipeline.tsePipeline;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red Warehouse FSM", group = "Dababy")
public class redWarehouseFSM extends LinearOpMode {

    private tsePipeline pipeline = new tsePipeline(COLOR.TSE,telemetry);
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    private Mechanisms mechanisms;
    private int freightCount = 0;
    private int desiredFreightCount = 2;

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

        TrajectorySequence movingToHub = chassis.trajectorySequenceBuilder(new Pose2d(10, -63, Math.toRadians(-90)))
                .addSpatialMarker(new Vector2d(5,-50),() -> { mechanisms.rotateArm(650,0.25); })
                .lineToSplineHeading(new Pose2d(-2,-40, Math.toRadians(-240))).build();
        TrajectorySequence movingToWarehouse = chassis.trajectorySequenceBuilder(movingToHub.end())
                .addSpatialMarker(new Vector2d(40,-64),() -> { mechanisms.moveIntake(0.7); })
                .setReversed(true).splineTo(new Vector2d(10, -60),Math.toRadians(-20))
                .addDisplacementMarker(() -> { mechanisms.reset(); })
                .splineTo(new Vector2d(50,-64),Math.toRadians(0)).setReversed(true).build();
        TrajectorySequence intakingFreight = chassis.trajectorySequenceBuilder(movingToWarehouse.end())
                .back(5).forward(5).back(5).forward(5).build();
        TrajectorySequence returningToHub = chassis.trajectorySequenceBuilder(intakingFreight.end())
                .addSpatialMarker(new Vector2d(0,-50),() -> {
                    mechanisms.rotateArm(650,0.25);
                }).splineTo(new Vector2d(-11.5,-41),Math.toRadians(90)).build();

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

        while(!isStopRequested()) {
            location = pipeline.getLocation();
            telemetry.addData("location",location);
            telemetry.update();
        }

        waitForStart();

        if (!opModeIsActive() || isStopRequested()) return;

        currentState = AUTO_STATE.MOVING_TO_HUB;
        chassis.followTrajectorySequenceAsync(movingToHub);

        masterLoop: while (opModeIsActive() && !isStopRequested()) {
            switch(currentState) {
                case MOVING_TO_HUB:
                case RETURNING_TO_HUB: {
                    if (!chassis.isBusy()) {
                        currentState = AUTO_STATE.DELIVERING_FREIGHT;
                    }
                    break;
                }
                case DELIVERING_FREIGHT: {
                    mechanisms.releaseServo.setPosition(0.3);
                    if(mechanisms.releaseServo.getPosition() <= 0.31) {
                        freightCount++;
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                        chassis.followTrajectorySequenceAsync(movingToWarehouse);
                    }
                    break;
                }
                case MOVING_TO_WAREHOUSE: {
                    if(!chassis.isBusy() && freightCount <= desiredFreightCount) {
                        currentState = AUTO_STATE.INTAKING_FREIGHT;
                        chassis.followTrajectorySequenceAsync(intakingFreight);
                    }
                    else {
                        currentState = AUTO_STATE.PATH_FINISHED;
                    }
                    break;
                }
                case INTAKING_FREIGHT: {
                    if (!chassis.isBusy()) {
                        currentState = AUTO_STATE.RETURNING_TO_HUB;
                        chassis.followTrajectorySequence(returningToHub);
                    }
                    break;
                }
                case PATH_FINISHED: {
                    break masterLoop;
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

        telemetry.addData("Master","Path Finished");
        telemetry.update();
    }
}