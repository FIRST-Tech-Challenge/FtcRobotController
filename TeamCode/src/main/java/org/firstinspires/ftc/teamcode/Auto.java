package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="autonomous")
public class Auto extends LinearOpMode {

    public class Coordinates{
        Pose2d startPose = new Pose2d(-63, 12, Math.toRadians(180));
        Pose2d rightTeamProp = new Pose2d(-35, 9.5, Math.toRadians(90.00));
        Vector2d centerTeamProp = new Vector2d(-32, 12);
        Vector2d leftTeamProp = new Vector2d(-42, 20);
        Pose2d leftBackdropIntermediate = new Pose2d(-36, 30, Math.toRadians(270));
        Pose2d centerBackdropIntermediate = new Pose2d(-36, 30, Math.toRadians(270));
        Pose2d rightBackdropIntermediate = new Pose2d(-36, 30, Math.toRadians(270));
        Pose2d leftBackdrop = new Pose2d(-36, 48, Math.toRadians(270.00));
        Pose2d centerBackdrop = new Pose2d(-36, 48, Math.toRadians(270.00));
        Pose2d rightBackdrop = new Pose2d(-36, 48, Math.toRadians(270.00));

    }
    Coordinates c = new Coordinates();
    static final double SLOWERVELOCITY = 15;
    static final double SLOWERANGULARVELOCITY = 2.5;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // hardware map for odometry encoders
        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, null, null);
        // start location (coordinate)

        //We must never forget these two lines of code ever again as per midnight of 11/26/2023
        myLocalizer.setPoseEstimate(c.startPose);
        drive.setPoseEstimate(c.startPose);

        // hardware map to get motors and sensors
        TrajectorySequence leftSpikeClose = drive.trajectorySequenceBuilder(c.startPose)
                //.lineTo(c.leftTeamProp)
                //.lineTo(c.centerTeamProp)
                .lineToLinearHeading(c.rightTeamProp)
                .back(3.5)
                .lineToLinearHeading(c.centerBackdropIntermediate)
                .lineToLinearHeading(c.centerBackdrop, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        waitForStart();
        drive.followTrajectorySequence(leftSpikeClose);
    }
}
