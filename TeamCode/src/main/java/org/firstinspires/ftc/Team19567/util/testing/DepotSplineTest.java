package org.firstinspires.ftc.Team19567.util.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.MecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.pipeline.greenPipeline;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.Team19567.util.Utility_Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * useless garbage, do not bother with this
 */
@Autonomous(name="Depot Spline Test", group="Testing")
@Disabled
@Deprecated
public class DepotSplineTest extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private ElapsedTime carouselTimeout = new ElapsedTime();
    private greenPipeline pipeline = new greenPipeline(telemetry); //Team shipping element OpenCV Pipeline
    private TouchSensor limitSwitch = null;
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    private AUTO_STATE currentState = AUTO_STATE.DETECTING_OPENCV;
    private int chosenArmPos = 600;
    private double chosenArmSpeed = 0.3;
    private double chosenTrajectoryX = -1;
    private Mechanisms mechanisms = null;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MecanumDriveCancelable chassis = new MecanumDriveCancelable(hardwareMap);

        mechanisms = new Mechanisms(hardwareMap,telemetry);
        mechanisms.setModes();

        limitSwitch = hardwareMap.get(TouchSensor.class,"limitSwitch");

        chassis.setPoseEstimate(new Pose2d(-34, -63, Math.toRadians(90)));

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
            telemetry.addData("Location",location);
            telemetry.update();
        }
        if(!opModeIsActive() || isStopRequested()) return;

        waitForStart();

        switch(location) {
            case ALLIANCE_FIRST: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = -32.5;
                telemetry.addData("OpenCV","Essebtuakky Tgurd Level Detected");
                telemetry.update();
                break;
            }
            case NO_ALLIANCE: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = -32.5;
                telemetry.addData("OpenCV","Basically Third Level");
                telemetry.update();
                break;
            }
            case ALLIANCE_SECOND: {
                chosenArmPos = Utility_Constants.SECOND_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.SECOND_LEVEL_POWER;
                chosenTrajectoryX = -36;
                telemetry.addData("OpenCV","Actually Second Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_THIRD: {
                chosenArmPos = Utility_Constants.FIRST_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.FIRST_LEVEL_POWER;
                chosenTrajectoryX = -40;
                telemetry.addData("OpenCV","basiccly first Level Detected");
                telemetry.update();
                break;
            }
            default: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = -32.5;
                telemetry.addData("OpenCV","Essebtuakky Tgurd Level Detected");
                telemetry.update();
            }
        }
        currentState = AUTO_STATE.MOVING_TO_HUB;

        TrajectorySequence preloadSequence = chassis.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                .addSpatialMarker(new Vector2d(-33,-450),() -> {
                    mechanisms.rotateArm(chosenArmPos,chosenArmSpeed);
                }).addSpatialMarker( new Vector2d(-32, -36), () -> {
                    mechanisms.releaseServoMove(0.2);
                }).lineToSplineHeading(new Pose2d(chosenTrajectoryX,-24,Math.toRadians(180)))
                .build();
        TrajectorySequence moveToCarouselSequence = chassis.trajectorySequenceBuilder(preloadSequence.end())
                .lineToSplineHeading(new Pose2d(-64,-57,Math.toRadians(0))).build();
        chassis.followTrajectorySequenceAsync(moveToCarouselSequence);
        TrajectorySequence warehouseSequence = chassis.trajectorySequenceBuilder(moveToCarouselSequence.end())
                .splineToConstantHeading(new Vector2d(15,-35),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(45,-35),Math.toRadians(180)).build();
        chassis.followTrajectorySequenceAsync(warehouseSequence);
        chassis.followTrajectorySequenceAsync(preloadSequence);

        master:while(opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = chassis.getPoseEstimate();
            telemetry.addData("Pose X",poseEstimate.getX());
            telemetry.addData("Pose Y",poseEstimate.getY());
            telemetry.addData("Pose Heading",poseEstimate.getHeading());

            switch(currentState) {
                case MOVING_TO_HUB: {
                    mechanisms.moveIntake(0);
                    if(!chassis.isBusy()) {
                        timeout.reset();
                        telemetry.addData("State Machine","Moved to DELIVERING_FREIGHT");
                        telemetry.update();
                        currentState = AUTO_STATE.DELIVERING_FREIGHT;
                        //mechanisms.releaseServoMove(0.3);
                    }
                    break;
                }
                case DELIVERING_FREIGHT: {
                    if(timeout.milliseconds() >= Utility_Constants.FLICKER_TIME) {
                        telemetry.addData("State Machine","Moved to MOVING_TO_CAROUSEL");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_CAROUSEL;
                        chassis.followTrajectorySequenceAsync(moveToCarouselSequence);
                    }
                    break;
                }
                case MOVING_TO_CAROUSEL: {
                    mechanisms.reset();
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to ROTATING_CAROUSEL");
                        telemetry.update();
                        carouselTimeout.reset();
                        currentState = AUTO_STATE.ROTATING_CAROUSEL;
                    }
                    break;
                }
                case ROTATING_CAROUSEL: {
                    mechanisms.rotateCarousel(Utility_Constants.INIT_POWER);
                    if(carouselTimeout.milliseconds() >= Utility_Constants.MILLI_END+500) {
                        telemetry.addData("State Machine","Moved to ROTATING_CAROUSEL");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                        chassis.followTrajectorySequenceAsync(warehouseSequence);
                    }
                    break;
                }
                case MOVING_TO_WAREHOUSE: {
                    mechanisms.rotateCarousel(0.0);
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to PATH_FINISHED");
                        telemetry.update();
                        currentState = AUTO_STATE.PATH_FINISHED;
                    }
                    break;
                }
                case PATH_FINISHED: {
                    break master;
                }
                default: {
                    currentState = AUTO_STATE.MOVING_TO_HUB;
                }
            }
            if(!limitSwitch.isPressed()) {
                mechanisms.moveIntake(0.1);
            }
            else {
                mechanisms.moveIntake(0.0);
            }
            mechanisms.maintainBalance();
            chassis.update();
            telemetry.addData("State", currentState);
            telemetry.update();
        }

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}