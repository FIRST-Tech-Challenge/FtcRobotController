package org.firstinspires.ftc.Team19567.util.testing;

import android.text.method.Touch;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.pipeline.greenPipeline;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.Team19567.util.Utility_Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Extended Spline Test", group="Testing")

public class SplineTest extends LinearOpMode {
    private Mechanisms mechanisms = null;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDriveCancelable chassis = new SampleMecanumDriveCancelable(hardwareMap);

        mechanisms = new Mechanisms(hardwareMap,telemetry);
        mechanisms.setModes();

        TrajectorySequence splineSequence = chassis.trajectorySequenceBuilder(new Pose2d(10,-63,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-2,-40,Math.toRadians(-45)))
                .build();

        TrajectorySequence returnSplineSequence = chassis.trajectorySequenceBuilder(splineSequence.end())
                .splineTo(new Vector2d(16, -64),Math.toRadians(-10))
                .splineTo(new Vector2d(54,-66.5),Math.toRadians(0))
                .build();

        TrajectorySequence hubSplineSequence = chassis.trajectorySequenceBuilder(returnSplineSequence.end())
                .setReversed(true).splineTo(new Vector2d(15, -68),Math.toRadians(170)).splineTo(new Vector2d(8,-39),Math.toRadians(135))
                .setReversed(false).build();

        //Lol
        TrajectorySequence fullSplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(10,-63,Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-2,-40,Math.toRadians(-45)))
                .splineTo(new Vector2d(16, -64),Math.toRadians(-10))
                .splineTo(new Vector2d(54,-66.5),Math.toRadians(0))
                .setReversed(true).splineTo(new Vector2d(15, -68),Math.toRadians(170)).splineTo(new Vector2d(8,-39),Math.toRadians(135))
                .setReversed(false).splineTo(new Vector2d(16, -64),Math.toRadians(-10))
                .splineTo(new Vector2d(54,-66.5),Math.toRadians(0))
                .setReversed(true).splineTo(new Vector2d(15, -68),Math.toRadians(170)).splineTo(new Vector2d(8,-39),Math.toRadians(135))
                .setReversed(false).splineTo(new Vector2d(16, -64),Math.toRadians(-10))
                .splineTo(new Vector2d(54,-66.5),Math.toRadians(0))
                .setReversed(true).splineTo(new Vector2d(15, -68),Math.toRadians(170)).splineTo(new Vector2d(8,-39),Math.toRadians(135))
                .setReversed(false)
                .build();

        chassis.setPoseEstimate(splineSequence.start());

        waitForStart();
        if(isStopRequested()) return;

        chassis.followTrajectorySequence(fullSplineSequence);

        /*
        chassis.followTrajectorySequenceAsync(splineSequence);

        while(opModeIsActive() && !isStopRequested()) {

        }
        */

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}