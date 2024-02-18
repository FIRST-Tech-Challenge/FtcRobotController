package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "red far", group = "Concept")
public class AutoTemplate2 extends LinearOpMode{
    // tensorflow object detection
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {"box"};
    private TfodProcessor tfod;
    String detection; // variable for position of model (left, middle, or right)

//    AprilTag IDs
//    blue:
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//    red:
//    int LEFT = 4;
//    int MIDDLE = 5;
//    int RIGHT = 6;

    // lift control
    int targetPosition = 1;
    int loop = 0;
    int currentPosition;
    boolean direction = true; //true == up
    boolean done = false;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    double xCoordinate;
    double yCoordinate;

    @Override
    public void runOpMode() { // code to run after init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Tfod processor
        initTfod();
        tfod.setMinResultConfidence((float) 0.65);
        tfod.setZoom(1.1);

        // create visionportal with two processors
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(drive.camera, tfod); // initialize visionportal

        // starting position
        Pose2d startingPose = new Pose2d();
        drive.setPoseEstimate(startingPose);

        /*
        TrajectorySequence beginning = drive.trajectorySequenceBuilder(startingPose)
                .addTemporalMarker(() -> {
                    drive.doorServo.setPosition(0.7);
                })
                .lineToLinearHeading(new Pose2d(33, 0, Math.toRadians(90)))
                .forward(5)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    targetPosition = 1300;
                })
                .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(-90)))
                .waitSeconds(.1)
                .addTemporalMarker(() -> {
                    drive.rightLiftServo.setPosition(1);
                })
                .waitSeconds(.1)
                .forward(6)
                .addTemporalMarker(() -> {
                    drive.doorServo.setPosition(0);
                })
                .waitSeconds(1.25)
                .back(10)
                .addTemporalMarker(() -> {
                    drive.rightLiftServo.setPosition(0.43);
                })
                .waitSeconds(.1)
                .addTemporalMarker(() -> {
                    direction = false;
                    targetPosition = 0;
                })
                .build();
        drive.followTrajectorySequenceAsync(beginning);

         */

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            updateTfod();// Push telemetry to the Driver Station.
            drive.pixelServo.setPosition(0);
            telemetry.update();
        }

        visionPortal.close();
        visionPortal.stopStreaming();

        waitForStart();

        while (opModeIsActive()) {
            if (detection == "left" && !drive.isBusy() && loop == 0) {
                drive.setPoseEstimate(startingPose);
                TrajectorySequence left = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            drive.doorServo.setPosition(0.7);
                        })
                        .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(90)))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(3, -5, Math.toRadians(90)))
                        .back(50)
                        .addTemporalMarker(() -> {
                            direction = true;
                            targetPosition = 1750;
                        })
                        .lineToLinearHeading(new Pose2d(36, -84, Math.toRadians(-90)))
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> {
                            drive.rightLiftServo.setPosition(1);
                        })
                        .waitSeconds(.25)
                        .forward(7)
                        .addTemporalMarker(() -> {
                            drive.doorServo.setPosition(0);
                        })
                        .waitSeconds(1.5)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            targetPosition = 2300;
                            direction = true;
                        })
                        .waitSeconds(.1)
                        .back(10)
                        .waitSeconds(.2)
                        .addTemporalMarker(() -> {
                            drive.rightLiftServo.setPosition(0.43);
                        })
                        .waitSeconds(.1)
                        .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                            targetPosition = 100;
                            direction = false;
                        })
                        .strafeLeft(30)
                        .forward(17)
                        .waitSeconds(20)
                        .build();
                drive.followTrajectorySequenceAsync(left);
                loop++;
            }
            else if (detection == "middle" && !drive.isBusy() && loop == 0) {
                TrajectorySequence middle = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            drive.doorServo.setPosition(0.7);
                        })
                        .forward(32)
                        .back(10)
                        .lineToLinearHeading(new Pose2d(3, 0, Math.toRadians(-90)))
                        .forward(50)
                        .addTemporalMarker(() -> {
                            direction = true;
                            targetPosition = 1750;
                        })
                        .lineToLinearHeading(new Pose2d(30, -84, Math.toRadians(-90)))
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> {
                            drive.rightLiftServo.setPosition(1);
                        })
                        .waitSeconds(.25)
                        .forward(7)
                        .addTemporalMarker(() -> {
                            drive.doorServo.setPosition(0);
                        })
                        .waitSeconds(1.5)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            targetPosition = 2300;
                            direction = true;
                        })
                        .waitSeconds(.1)
                        .back(10)
                        .waitSeconds(.2)
                        .addTemporalMarker(() -> {
                            drive.rightLiftServo.setPosition(0.43);
                        })
                        .waitSeconds(.1)
                        .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                            targetPosition = 100;
                            direction = false;
                        })
                        .strafeLeft(30)
                        .forward(17)
                        .waitSeconds(20)
                        .build();
                drive.followTrajectorySequenceAsync(middle);
                loop++;
            }
            else if (detection == "right" && !drive.isBusy() && loop == 0) {
                TrajectorySequence right = drive.trajectorySequenceBuilder(startingPose)
                        .addTemporalMarker(() -> {
                            drive.doorServo.setPosition(0.7);
                        })
                        .forward(15)
                        .lineToLinearHeading(new Pose2d(37, -2, Math.toRadians(-90)))
                        .waitSeconds(0.5)
                        .lineToLinearHeading(new Pose2d(3, 5, Math.toRadians(-90)))
                        .forward(54)
                        .addTemporalMarker(() -> {
                            direction = true;
                            targetPosition = 1750;
                        })
                        .lineToLinearHeading(new Pose2d(25, -84, Math.toRadians(-90)))
                        .waitSeconds(.1)
                        .addTemporalMarker(() -> {
                            drive.rightLiftServo.setPosition(1);
                        })
                        .waitSeconds(.25)
                        .forward(7)
                        .addTemporalMarker(() -> {
                            drive.doorServo.setPosition(0);
                        })
                        .waitSeconds(1.5)
                        .UNSTABLE_addDisplacementMarkerOffset(0, () -> {
                            targetPosition = 2300;
                            direction = true;
                        })
                        .waitSeconds(.1)
                        .back(10)
                        .waitSeconds(.2)
                        .addTemporalMarker(() -> {
                            drive.rightLiftServo.setPosition(0.43);
                        })
                        .waitSeconds(.1)
                        .UNSTABLE_addDisplacementMarkerOffset(3, () -> {
                            targetPosition = 100;
                            direction = false;
                        })
                        .strafeLeft(35)
                        .forward(17)
                        .waitSeconds(20)
                        .build();
                drive.followTrajectorySequenceAsync(right);
                loop++;
            }
            if (!drive.isBusy() && loop == 1) {
                break;
            }
            drive.update();
            liftUpdate(drive);
        }
    }

    private void initTfod() { // create model detector
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName("model.tflite")
                .setMaxNumRecognitions(1)
                .build();
    }

    private void updateTfod() { // updates the model detection and add to telemetry
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            xCoordinate = x;
            yCoordinate = y;
            if (recognition.getRight() - recognition.getLeft() < 200) {
                if (xCoordinate < 250) {
                    detection = "left";
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f / %s", x, y, detection);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    return;
                }
                else if (xCoordinate < 550) {
                    detection = "middle";
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f / %s", x, y, detection);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    return;
                }
            }
            detection = "right";
        }
    }

    // update lift position - constantly called during opmode
    public void liftUpdate(SampleMecanumDrive drive) {
        currentPosition = drive.liftMotor1.getCurrentPosition();
        if (targetPosition == 0){
        }
        else if (currentPosition < targetPosition && direction == true) {
            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
            drive.liftMotor1.setPower(power);
            drive.liftMotor2.setPower(power);

        } else if (currentPosition > targetPosition && direction == false) {
            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
            drive.liftMotor1.setPower(power);
            drive.liftMotor2.setPower(power);
        }
        else if (currentPosition+10 > targetPosition && direction == true){
            drive.liftMotor1.setPower(0.05);
            drive.liftMotor2.setPower(0.05);
        }
        else if (currentPosition+10 < targetPosition && direction == false){
            drive.liftMotor1.setPower(0.05);
            drive.liftMotor2.setPower(0.05);
        }
    }

    // lift control for getting power for motors
    public double returnPower(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
        return output;
    }
}