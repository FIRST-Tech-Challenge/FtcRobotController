package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous (name="LEFT SIDE")
public class League2LeftSideAuto extends BaseAutonomous {

    @Override
    public void runOpMode() {

        initializeAuto();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory traject2 = drive.trajectoryBuilder(startPose, false)
                .forward(48)
                .build();
        Trajectory traject3 = drive.trajectoryBuilder(traject2.end(), false)
                .back(21)
                .build();
        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .strafeLeft(18.5)
                .build();
        Trajectory right = drive.trajectoryBuilder(traject4.end(), false)
                .strafeRight(45)
                .build();
        Trajectory middle = drive.trajectoryBuilder(traject4.end(), false)
                .strafeRight(21)
                .build();
        Trajectory left = drive.trajectoryBuilder(traject4.end(), false)
                .strafeLeft(12)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            detectAprilTag();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        updateTelemetryAfterStart();

        grabberServo.setPosition(GRABBER_CLOSED);
        raiseAndHoldArmGroundJunctionPosition();

        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);

        raiseAndHoldArmLowJunctionPosition();
        drive.followTrajectory(traject4);
        motorArm.setPower(0);
        sleep(800);
        grabberServo.setPosition(GRABBER_OPEN);

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(left);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(middle);
        } else {
            drive.followTrajectory(right);
        }
    }
}