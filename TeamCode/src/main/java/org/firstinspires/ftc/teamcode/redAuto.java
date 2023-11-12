package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class redAuto extends LinearOpMode {
    Hardware robot = new Hardware();
    private propPositions propPosition;

    public enum propPositions {
        LEFT,
        RIGHT,
        CENTER
    }

    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        robot.gripper.setPosition(0.55);
        robot.arm.setPosition(1);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory initial = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-6, 23))
                .build();
        Trajectory centerPlace = drive.trajectoryBuilder(initial.end())
                .lineToConstantHeading(new Vector2d(-6,15))
                .splineToSplineHeading(new Pose2d(36,28, 0), Math.toRadians(0))
                .build();
        Trajectory rightPlace = drive.trajectoryBuilder(initial.end())
                .lineToConstantHeading(new Vector2d(2,10))
                .splineToSplineHeading(new Pose2d(37,18, 0), Math.toRadians(0))
                .build();
        Trajectory leftPlace = drive.trajectoryBuilder(initial.end())
                .lineToConstantHeading(new Vector2d(2,10))
                .splineToSplineHeading(new Pose2d(37,32, 0), Math.toRadians(0))
                .build();

        telemetry.addData("Robot Ready For Start", "");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(initial);

        sleep(1000);

        if (robot.rightDistance.getDistance(DistanceUnit.CM) > 5 && robot.rightDistance.getDistance(DistanceUnit.CM) < 15) {
            propPosition = propPositions.RIGHT;
        } else if (robot.leftDistance.getDistance(DistanceUnit.CM) > 5 && robot.leftDistance.getDistance(DistanceUnit.CM) < 15) {
            propPosition = propPositions.LEFT;
        } else {
            propPosition = propPositions.CENTER;
        }

        telemetry.addData("Position", propPosition);
        telemetry.addData("Right Distance", robot.rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", robot.leftDistance.getDistance(DistanceUnit.CM));
        telemetry.update();

        switch (propPosition) {
            case LEFT: {
                drive.followTrajectory(leftPlace);
                robot.arm.setPosition(.4);
                sleep(2000);
                robot.gripper.setPosition(1);
                sleep(2000);
                robot.arm.setPosition(1);
                sleep(2000);
                robot.gripper.setPosition(0.7);
                return;
            }
            case RIGHT: {
                drive.followTrajectory(rightPlace);
                robot.arm.setPosition(.4);
                sleep(2000);
                robot.gripper.setPosition(1);
                sleep(2000);
                robot.arm.setPosition(1);
                sleep(2000);
                robot.gripper.setPosition(0.7);
                return;
            }
            case CENTER: {
                drive.followTrajectory(centerPlace);
                robot.arm.setPosition(.4);
                sleep(2000);
                robot.gripper.setPosition(1);
                sleep(2000);
                robot.arm.setPosition(1);
                sleep(2000);
                robot.gripper.setPosition(0.7);
                return;
            }
        }
    }
}
