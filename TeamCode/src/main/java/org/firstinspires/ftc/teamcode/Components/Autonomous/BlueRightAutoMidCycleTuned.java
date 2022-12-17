package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_MED_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "BlueRightAutoMidCycleTuned")


public class BlueRightAutoMidCycleTuned extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dropX = -31, dropY = 16.5 , dropA = toRadians(210), dropET = toRadians(30);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -62, pickupY2 = 10.5, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    double[] stackPos = {390, 290, 200, 80, 0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-34, 46, toRadians(90)), toRadians(270))
                .splineToSplineHeading(new Pose2d(-30.5, 29, toRadians(135)), toRadians(270))
                .build();
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-30.5,29, toRadians(135)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34,11.5, toRadians(180)),toRadians(240))
                .splineToSplineHeading(new Pose2d(pickupX2, pickupY2, pickupA2), pickupET2)
                .build();
        TrajectorySequence dropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2, pickupY2, pickupA2))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(dropX,dropY, dropA), dropET)
                .build();
        //SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_ACCEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
        TrajectorySequence pickupTrajectory2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-62, 10.5, toRadians(180)), toRadians(180))
                .build();
        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .splineToSplineHeading(new Pose2d(-36, 33, toRadians(90)), toRadians(90))
                .build();
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-50, 10, toRadians(180)), toRadians(180))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-10, 10, toRadians(180)), toRadians(180))
                .build();

        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dropX,dropY, dropA))
                .splineToSplineHeading(new Pose2d(-35, 16, toRadians(180)), toRadians(180))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dropX,dropY, dropA))
                .splineToSplineHeading(new Pose2d(-56, 12, toRadians(180)), toRadians(180))

                .build();
        while (!isStarted()) {
            telemetry.addData("pos", robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateLiftArmStates();

        }
        resetRuntime();
        dummyP = robot.cv.getPosition();

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested() && getRuntime() < 29) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(initialtrajectory);
            robot.delay(0.2);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.9);
            robot.liftToPosition(LIFT_MED_JUNCTION);
            robot.waitForFinish();
            robot.openClaw(false);
            robot.cycleLiftArmToCycle(true);
            robot.delay(1.5);
            robot.wideClaw();
            robot.delay(0.2);
            robot.liftToPosition((int) stackPos[0]);
            robot.followTrajectorySequenceAsync(pickupTrajectory);
            robot.waitForFinish();
            robot.closeClaw(false);
            robot.waitForFinish();
            robot.delay(0.2);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.9);
            robot.liftToPosition(LIFT_MED_JUNCTION);
            robot.followTrajectorySequenceAsync(dropTrajectory);
            robot.waitForFinish();
            robot.openClaw();
            robot.waitForFinish();
            for (int i = 0; i < 3; i++) {
                robot.delay(0.4);
                robot.cycleLiftArmToCycle(true);
                robot.delay(1.4);
                robot.wideClaw();
                robot.delay(0.7);
                robot.liftToPosition((int) stackPos[i + 1]);
                robot.delay(0.4);
                robot.followTrajectorySequenceAsync(pickupTrajectory2);
                robot.waitForFinish();
                robot.closeClaw(false);
                robot.waitForFinish();
                robot.delay(0.2);
                robot.raiseLiftArmToOuttake(true);
                robot.delay(1.0);
                robot.liftToPosition(LIFT_MED_JUNCTION);
                robot.followTrajectorySequenceAsync(dropTrajectory);
                robot.waitForFinish();
                robot.openClaw();
                robot.waitForFinish();
            }
//            robot.followTrajectorySequenceAsync(parkTrajectory);
//            robot.delay(1);
            robot.delay(0.5);
            robot.liftToPosition(0);
            robot.delay(0.5);
            robot.lowerLiftArmToIntake(true);

            if (dummyP == 1) {
//                robot.delay(0.5);
                robot.followTrajectorySequenceAsync(park1trajectory);
            } else if (dummyP == 3) {
                robot.delay(0.5);
                robot.followTrajectoryAsync(park3trajectory);
            } else {
                robot.delay(0.5);
                robot.followTrajectoryAsync(park2trajectory);
            }

            robot.setFirstLoop(false);
            robot.liftToTargetAuto();
            robot.roadrun.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();

        }
        robot.stop();
        if (getRuntime() > 29.8) {
            stop();
        }
    }
}
