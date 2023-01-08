package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Interleague Left")
public class InterleagueAutoLeft extends BaseAutonomous {
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
                .back(32)
                .build();
        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .strafeRight(16)
                .build();
        Trajectory traject5 = drive.trajectoryBuilder(traject4.end(), false)
                .back(3)
                .build();
        Trajectory left = drive.trajectoryBuilder(traject5.end(), false)
                .strafeLeft(48)
                .build();
        Trajectory middle = drive.trajectoryBuilder(traject5.end(), false)
                .strafeLeft(18)
                .build();
        Trajectory right = drive.trajectoryBuilder(traject5.end(), false)
                .strafeRight(12)
                .build();
        Trajectory traject6 = drive.trajectoryBuilder(traject5.end(), false)
                .strafeRight(19)
                .build();
        Trajectory traject7 = drive.trajectoryBuilder(traject6.end(), false)
                .forward(32)
                .build();
        Trajectory traject8 = drive.trajectoryBuilder(traject7.end().plus(new Pose2d(0,0, Math.toRadians(-90))), false)
                .forward(22)
                .build();
        Trajectory traject9 = drive.trajectoryBuilder(traject8.end(), false)
                .forward(4)
                .build();
        Trajectory traject10 = drive.trajectoryBuilder(traject9.end(), false)
                .back(20)
                .build();

        while (!isStarted() && !isStopRequested()) {
            detectAprilTag();
        }
        updateTelemetryAfterStart();

        grabberServo.setPosition(0.4);
        drive.followTrajectory(traject1);
        while (Math.abs(motorArm.getCurrentPosition() - GRD_JUNCT_ARM_POSITION) > 10 && opModeIsActive()) {
            motorArm.setPower((GRD_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) / 400.0);
        }
        motorArm.setPower(0.01);

        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);

        motorArm.setPower(0);
        while ((Math.abs(motorArm.getCurrentPosition() - (MID_JUNCT_ARM_POSITION - 50)) > 20) && opModeIsActive()) {
            motorArm.setPower((MID_JUNCT_ARM_POSITION - 50 - motorArm.getCurrentPosition()) / 600.0);

        }
        motorArm.setPower(0.005);
        drive.followTrajectory(traject4);
        motorArm.setPower(0);
        sleep(1300);
        // open servo
        grabberServo.setPosition(0.8);
        drive.followTrajectory(traject5);

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(left);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(middle);
        } else {
            drive.followTrajectory(right);
        }

        //drive.followTrajectory(traject5);

        /*
        drive.followTrajectory(traject6);
        drive.followTrajectory(traject7);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(traject8);
        while ((Math.abs(motorArm.getCurrentPosition() - 400) > 20) && opModeIsActive()) {
            motorArm.setPower((400 - motorArm.getCurrentPosition()) / 500.0);
        }
        motorArm.setPower(0.005);
        drive.followTrajectory(traject9);
        grabberServo.setPosition(GRABBER_CLOSED);
        sleep(500);
        while ((Math.abs(motorArm.getCurrentPosition() - (MID_JUNCT_ARM_POSITION)) > 20) && opModeIsActive()) {
            motorArm.setPower((MID_JUNCT_ARM_POSITION - motorArm.getCurrentPosition()) / 600.0);

        }
        motorArm.setPower(0.005);
        drive.followTrajectory(traject10);
        motorArm.setPower(0);*/


    }
}
