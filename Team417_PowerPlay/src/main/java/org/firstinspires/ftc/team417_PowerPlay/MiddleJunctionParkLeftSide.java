package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Interleague Left")
public class MiddleJunctionParkLeftSide extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto();

        // declare trajectories here
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory clearWall = drive.trajectoryBuilder(startPose, false)
                .forward(5)
                .build();
        Trajectory pushSignalCone = drive.trajectoryBuilder(clearWall.end(), false)
                .forward(40)
                .build();
        Trajectory backToMidJunction = drive.trajectoryBuilder(pushSignalCone.end(), false)
                .back(15)
                .build();
        Trajectory rightToMidJunction = drive.trajectoryBuilder(backToMidJunction.end(), false)
                .strafeRight(14)
                .build();
        Trajectory clearJunction = drive.trajectoryBuilder(rightToMidJunction.end(), false)
                .back(3)
                .build();
        Trajectory parkLeft = drive.trajectoryBuilder(clearJunction.end(), false)
                .strafeLeft(44)
                .build();
        Trajectory parkMiddle = drive.trajectoryBuilder(clearJunction.end(), false)
                .strafeLeft(18)
                .build();
        Trajectory parkRight = drive.trajectoryBuilder(clearJunction.end(), false)
                .strafeRight(12)
                .build();

        while (!isStarted() && !isStopRequested()) {
            detectAprilTag();
        }
        updateTelemetryAfterStart();

        grabberServo.setPosition(0.4);
        drive.followTrajectory(clearWall);
        raiseAndHoldArmGroundJunctionPosition();

        drive.followTrajectory(pushSignalCone);
        drive.followTrajectory(backToMidJunction);

        raiseAndHoldArmMiddleJunctionPosition();
        drive.followTrajectory(rightToMidJunction);
        motorArm.setPower(0);
        sleep(2000);

        // open servo
        grabberServo.setPosition(GRABBER_OPEN);

        drive.followTrajectory(clearJunction);

        // check if tagOfInterest is null first to avoid null pointer exception when accessing id
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(parkLeft);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(parkMiddle);
        } else {
            drive.followTrajectory(parkRight);
        }
    }
}
