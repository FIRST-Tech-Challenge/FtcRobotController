package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Interleague Right")
public class InterleagueAutoRight extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeAuto();

        // declare trajectories here
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory traject1 = drive.trajectoryBuilder(startPose, false)
                .forward(5)
                .build();
        Trajectory traject2 = drive.trajectoryBuilder(traject1.end(), false)
                .forward(55)
                .build();
        Trajectory traject3 = drive.trajectoryBuilder(traject2.end(), false)
                .back(31.5)
                .build();
        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .strafeLeft(19)
                .build();
        Trajectory traject5 = drive.trajectoryBuilder(traject4.end(), false)
                .back(3)
                .build();
        Trajectory left = drive.trajectoryBuilder(traject5.end(), false)
                .strafeLeft(12)
                .build();
        Trajectory middle = drive.trajectoryBuilder(traject5.end(), false)
                .strafeRight(22)
                .build();
        Trajectory right = drive.trajectoryBuilder(traject5.end(), false)
                .strafeRight(48)
                .build();

        while (!isStarted() && !isStopRequested()) {
            detectAprilTag();
        }
        updateTelemetryAfterStart();

        grabberServo.setPosition(GRABBER_CLOSED);
        drive.followTrajectory(traject1);
        raiseAndHoldArmGroundJunctionPosition();

        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);

        raiseAndHoldArmMiddleJunctionPosition();
        drive.followTrajectory(traject4);
        motorArm.setPower(0);
        sleep(1300);
        // open servo
        grabberServo.setPosition(GRABBER_OPEN);
        drive.followTrajectory(traject5);

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(left);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(middle);
        } else {
            drive.followTrajectory(right);
        }
    }
}
