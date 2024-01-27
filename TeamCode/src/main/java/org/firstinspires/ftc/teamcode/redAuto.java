package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;


@Autonomous(name = "Red Auto", group = "Auto")
public class redAuto extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = new Hardware();
    private propPositions propPosition;
    private static int DESIRED_TAG_ID;
    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected

    public enum propPositions {
        LEFT,
        RIGHT,
        CENTER
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        initAprilTag();
        robot.droneAngle.setPosition(robot.droneAngleDown);
        robot.launcherRelease.setPosition(robot.launchClosed);
        robot.stripper.setPosition(robot.stripperOpen);
        robot.hook.setPosition(robot.hookDown);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d globalPose = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory toDetection = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(7, 22),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20))
                .build();

        waitForStart();
        visionPortal.stopStreaming();
        runtime.reset();
        if (isStopRequested()) return;

        robot.intake.setPower(1);

        drive.followTrajectory(toDetection);

        if ((robot.rightDistance.getDistance(DistanceUnit.CM) > 10.5) && (robot.rightDistance.getDistance(DistanceUnit.CM) < 15)) {
            propPosition = redAuto.propPositions.RIGHT;
            DESIRED_TAG_ID = 6;
            telemetry.addData("Running:", "Right");
        } else if ((robot.leftDistance.getDistance(DistanceUnit.CM) > 8) && (robot.leftDistance.getDistance(DistanceUnit.CM) < 15)) {
            propPosition = redAuto.propPositions.LEFT;
            DESIRED_TAG_ID = 4;
            telemetry.addData("Running:", "Left");
        } else {
            propPosition = redAuto.propPositions.CENTER;
            DESIRED_TAG_ID = 5;
            telemetry.addData("Running:", "Center");
        }
        telemetry.update();
        switch (propPosition) {
            case CENTER:
                Trajectory placePixelCenter = drive.trajectoryBuilder(toDetection.end())
                        .strafeTo(new Vector2d(0, 22))
                        .build();
                Trajectory globalPositionCenter1 = drive.trajectoryBuilder(placePixelCenter.end())
                        .splineToConstantHeading(new Vector2d(-15, 18), 0)
                        .splineToConstantHeading(new Vector2d(-10,55),0)
                        .splineToConstantHeading(new Vector2d(10,45),0)
                        .build();
                Trajectory globalPositionCenter2 = drive.trajectoryBuilder(globalPositionCenter1.end())
                        .lineToLinearHeading(new Pose2d(35, 42, 0))
                        .build();
                drive.followTrajectory(placePixelCenter);
                robot.transfer.setPower(-.32);
                sleep(450);
                drive.followTrajectory(globalPositionCenter1);
                drive.followTrajectory(globalPositionCenter2);
                globalPose = globalPositionCenter2.end();
                break;
            case RIGHT:
                Trajectory placePixelRight = drive.trajectoryBuilder(toDetection.end())
                        .strafeTo(new Vector2d(21, 22))
                        .build();
                Trajectory underTruss = drive.trajectoryBuilder(placePixelRight.end())
                        .strafeTo(new Vector2d(35, 22))
                        .build();
                Trajectory throughTruss = drive.trajectoryBuilder(underTruss.end())
                        .lineTo(new Vector2d(35, 46))
                        .build();
                Trajectory globalPositionRight = drive.trajectoryBuilder(throughTruss.end())
                        .lineToLinearHeading(new Pose2d(35, 47, Math.toRadians(0)))
                        .build();
                drive.followTrajectory(placePixelRight);
                robot.transfer.setPower(-.32);
                sleep(450);
                drive.followTrajectory(underTruss);
                drive.followTrajectory(throughTruss);
                drive.followTrajectory(globalPositionRight);
                globalPose = globalPositionRight.end();
                break;
            case LEFT:
                Trajectory placePixelLeft = drive.trajectoryBuilder(toDetection.end())
                        .splineToConstantHeading(new Vector2d(-7, 15), 0)
                        .build();
                Trajectory lineUp = drive.trajectoryBuilder(placePixelLeft.end())
                                .strafeTo(new Vector2d(5,15))
                                        .build();
                Trajectory lineUpStageDoor = drive.trajectoryBuilder(lineUp.end())
                                .lineTo(new Vector2d(5, 50))
                                        .build();
                Trajectory globalPositionLeft = drive.trajectoryBuilder(lineUpStageDoor.end())
                                .lineToLinearHeading(new Pose2d(35,47, Math.toRadians(0)))
                                        .build();
                drive.followTrajectory(placePixelLeft);
                robot.transfer.setPower(-.32);
                sleep(450);
                drive.followTrajectory(lineUp);
                drive.followTrajectory(lineUpStageDoor);
                drive.followTrajectory(globalPositionLeft);
                globalPose = globalPositionLeft.end();
                break;
        }
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
        Trajectory throughStageDoor = drive.trajectoryBuilder(globalPose)
                .lineToConstantHeading(new Vector2d(55,45))
                .build();
        Trajectory nextToBackboard = drive.trajectoryBuilder(throughStageDoor.end())
                .lineToLinearHeading(new Pose2d(85,40, Math.PI))
                .build();
        Trajectory alignToBackboard = drive.trajectoryBuilder(nextToBackboard.end())
                .strafeTo(new Vector2d(85,10))
                .build();
        drive.followTrajectory(throughStageDoor);
        drive.followTrajectory(nextToBackboard);
        drive.followTrajectory(alignToBackboard);

        double timeOut = getRuntime();
        double y = 0;

        while (opModeIsActive() && !targetFound && getRuntime() < timeOut + 3) {
            visionPortal.resumeStreaming();
// Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        desiredTag = detection;
                        targetFound = true;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        y = 0;
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    y = 0;
                }
            }
        }

        visionPortal.stopStreaming();

        if (targetFound) {
            y = desiredTag.ftcPose.y;
        }

        Trajectory lineUpToTag = drive.trajectoryBuilder(alignToBackboard.end())
                .strafeTo(new Vector2d(85,15 + (-(desiredTag.ftcPose.x + 7))))
                .build();
        Trajectory placePosition = drive.trajectoryBuilder(lineUpToTag.end())
                        .back(Math.abs(y) - 6,
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(20))
                                .build();
        Trajectory park = drive.trajectoryBuilder(placePosition.end())
                .strafeRight(30)
                .build();
        if (targetFound) {
        drive.followTrajectory(lineUpToTag); }
        robot.lift.setTargetPosition(750);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(.7);
        drive.followTrajectory(placePosition);
        robot.stripper.setPosition(robot.stripperSecondRelease);
        sleep(1500);
        robot.lift.setTargetPosition(10);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(.3);
        drive.followTrajectory(park);
    }


    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(911.384, 911.384, 605.325, 382.676) /**Parameters for Arducam**/

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(robot.webcam);


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 800));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

}
