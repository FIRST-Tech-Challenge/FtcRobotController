package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous (name="LEFT SIDE")
public class LowJunctionParkLeftSide extends BaseAutonomous {

    @Override
    public void runOpMode() {

        initializeAuto();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory pushSignalCone = drive.trajectoryBuilder(startPose, false)
                .forward(48)
                .build();
        Trajectory backToLowJunction = drive.trajectoryBuilder(pushSignalCone.end(), false)
                .back(21)
                .build();
        Trajectory leftToLowJunction = drive.trajectoryBuilder(backToLowJunction.end(), false)
                .strafeLeft(18.5)
                .build();
        Trajectory parkRight = drive.trajectoryBuilder(leftToLowJunction.end(), false)
                .strafeRight(45)
                .build();
        Trajectory parkMiddle = drive.trajectoryBuilder(leftToLowJunction.end(), false)
                .strafeRight(21)
                .build();
        Trajectory parkLeft = drive.trajectoryBuilder(leftToLowJunction.end(), false)
                .strafeLeft(12)
                .build();

        while (!isStarted() && !isStopRequested()) {
            detectAprilTag();
        }

        updateTelemetryAfterStart();

        grabberServo.setPosition(GRABBER_CLOSED);

        raiseAndHoldArmGroundJunctionPosition();

        drive.followTrajectory(pushSignalCone);
        drive.followTrajectory(backToLowJunction);

        raiseAndHoldArmLowJunctionPosition();
        drive.followTrajectory(leftToLowJunction);
        motorArm.setPower(0);
        sleep(800);

        grabberServo.setPosition(GRABBER_OPEN);

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(parkLeft);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(parkMiddle);
        } else {
            drive.followTrajectory(parkRight);
        }
    }
}