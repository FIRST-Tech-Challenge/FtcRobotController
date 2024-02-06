package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

@Autonomous(name = "NewAuto", group = "Concept")
public class NewAuto extends LinearOpMode{
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {"box"};
    TfodProcessor tfod = TfodProcessor.easyCreateWithDefaults();

    String detection;

    double xCoordinate;
    double yCoordinate;

    @Override
    public void runOpMode() { // code to run after init
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(drive.camera, tfod);
        // starting position
        Pose2d startingPose = new Pose2d(12, -60, Math.toRadians(90));

        while (!isStarted() && !isStopRequested()) {
            updateTfod();// Push telemetry to the Driver Station.
            drive.pixelServo.setPosition(0);
            telemetry.update();
        }

        visionPortal.close();
        visionPortal.stopStreaming();

        waitForStart();

        while (opModeIsActive()) {
            drive.setPoseEstimate(startingPose);
            if (detection == "left") {
                TrajectorySequence left = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(5, -30), Math.toRadians(180))
                        .addTemporalMarker(() -> {
                            drive.pixelServo.setPosition(1);
                        })
                        .waitSeconds(1)
//                        .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(180)))
                        .back(40, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .build();
                drive.followTrajectorySequence(left);
                drive.breakFollowing();
                break;
            }
            else if (detection == "middle") {
                TrajectorySequence middle = drive.trajectorySequenceBuilder(startingPose)
                        .lineToLinearHeading(new Pose2d(12, -30, Math.toRadians(90)))
                        .addTemporalMarker(() -> {
                            drive.pixelServo.setPosition(0.5);
                        })
                        .waitSeconds(1)
                        .back(15, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(0)))
                        .build();
                drive.followTrajectorySequence(middle);
                drive.breakFollowing();
                break;
            }
            else {
                TrajectorySequence right = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(16, -30), Math.toRadians(0))
                        .addTemporalMarker(() -> {
                            drive.pixelServo.setPosition(1);
                        })
                        .waitSeconds(1)
                        .back(8, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .strafeRight(15, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .forward(40, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(60))
                        .build();
                drive.followTrajectorySequence(right);
                drive.breakFollowing();
                break;
            }
        }
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
            String label = recognition.getLabel();
            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f / %s", x, y, detection);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
        if (xCoordinate < 200) {
            detection = "left";
        }
        else if (xCoordinate < 500) {
            detection = "middle";
        }
        else {
            detection = "right";
        }
    }


//    public void liftUpdate(SampleMecanumDrive drive) {
//        currentPosition = drive.liftMotor1.getCurrentPosition();
//        if (targetPosition == 0){
//        }
//        else if (currentPosition < targetPosition && direction == true) {
//            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
//            drive.liftMotor1.setPower(power);
//            drive.liftMotor2.setPower(power);
//
//        } else if (currentPosition > targetPosition && direction == false) {
//            double power = returnPower(targetPosition, drive.liftMotor1.getCurrentPosition());
//            drive.liftMotor1.setPower(power);
//            drive.liftMotor2.setPower(power);
//
//        }
//        else if (currentPosition+10 > targetPosition && direction == true){
//            drive.liftMotor1.setPower(0.05);
//            drive.liftMotor2.setPower(0.05);
//        }
//        else if (currentPosition+10 < targetPosition && direction == false){
//            drive.liftMotor1.setPower(0.05);
//            drive.liftMotor2.setPower(0.05);
//        }
//    }

//    public double returnPower(double reference, double state) {
//        double error = reference - state;
//        double derivative = (error - lastError) / timer.seconds();
//        lastError = error;
//
//        double output = (error * 0.03) + (derivative * 0.0002) + 0.05;
//        return output;
//    }
}