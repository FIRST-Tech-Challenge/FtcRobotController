package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_MED_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;


@Config
@Autonomous(name = "BlueLeftAutoMidCycleTuned")


public class BlueLeftAutoMidCycleTuned extends LinearOpMode {

    public static double dummyP = 3;

    public static double dropX = 28.5, dropY = 21, dropA = toRadians(320), dropET = toRadians(150);

//    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
public static double pickupX2 = 63, pickupY2 = 11.5, pickupA2 = toRadians(0), pickupET2 = toRadians(0);

    double[] stackPos = {440, 330, 245, 100, 0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(33.9, 62.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();

        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(33.9, 62.25,
                        Math.toRadians(90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(34, 48, toRadians(90)))
                .splineToSplineHeading(new Pose2d(32, 14, toRadians(315)), toRadians(270))
                .lineToLinearHeading(new Pose2d(dropX, dropY, toRadians(315)))

//                .addTemporalMarker(robot::done)
                .build();

        ArrayList<TrajectorySequence> dropTrajectory = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2, pickupY2+0.25*i, pickupA2))
                    .setReversed(true)
                    .splineTo(new Vector2d(dropX,dropY+0.25*i), dropET)
//                    .splineTo(new Vector2d(dropX,dropY + 0.2*i), dropET, robot.roadrun.getVelocityConstraint(80, 9, 14) , robot.roadrun.getAccelerationConstraint(50))
//                    .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), dropET, robot.roadrun.getVelocityConstraint(70, 9, 14) , robot.roadrun.getAccelerationConstraint(45))
                    .addTemporalMarker(robot::done)
                    .build());
        }

        ArrayList<TrajectorySequence> pickupTrajectory2 = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            pickupTrajectory2.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX , dropY+0.25*i, dropA))
                    .setReversed(false)
//                    .splineTo(new Vector2d(pickupX2,pickupY2), pickupET2)
                    .splineTo(new Vector2d(pickupX2,pickupY2+0.25*i), pickupET2, robot.roadrun.getVelocityConstraint(50, 9, 14) , robot.roadrun.getAccelerationConstraint(60))
//                .addTemporalMarker(()->{robot.done(); robot.roadrun.breakFollowing();})
//                            .splineToSplineHeading(new Pose2d(pickupX2,pickupY2,pickupA2), pickupET2, robot.roadrun.getVelocityConstraint(70, 9, 14) , robot.roadrun.getAccelerationConstraint(45))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(60,16, toRadians(0)),toRadians(90))
//                .splineToLinearHeading(new Pose2d(60,20, toRadians(90)),toRadians(90))
                .build();

        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .lineToLinearHeading(new Pose2d(38, 10,dropA))
                .lineToLinearHeading(new Pose2d(38, 18,toRadians(90)))
                .build();
        TrajectorySequence park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(dropX+4,dropY-9, toRadians(-5)),toRadians(dropA))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(11,12, toRadians(0)))
                .build();

        while (!isStarted()) {
            telemetry.addData("pos", robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            if (getRuntime() > 3) {
                int color = robot.cv.getPosition();

                if (color == 1) {
                    robot.heartbeatRed();
                } else if (color == 2) {
                    robot.darkGreen();
                } else {
                    robot.blue();
                }
            }
            robot.updateClawStates();
            robot.updateLiftArmStates();
        }
        resetRuntime();
        dummyP = robot.cv.getPosition();
//        robot.cv.observeStick();

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested() && getRuntime() < 29.8) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(initialtrajectory);
            robot.delay(0.3);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.5);
            robot.liftToPosition(LIFT_MED_JUNCTION);
            robot.openClaw(false);
            robot.delay(0.4);
            robot.cycleLiftArmToCycle(true);
            robot.delay(0.5);
            robot.wideClaw();
            robot.delay(0.5);
            robot.iterateConestackUp();
            robot.followTrajectorySequenceAsync(pickupTrajectory2.get(0));
            robot.closeClaw(false);
            robot.followTrajectorySequenceAsync(dropTrajectory.get(0));
            robot.liftToPosition((int) LIFT_MED_JUNCTION.getValue(), true);
            robot.delay(0.55);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.25);
            robot.openClaw(false);
            for (int i = 0; i < 4; i++) {
                robot.followTrajectorySequenceAsync(pickupTrajectory2.get(i));
                if (i != 3) {
                    robot.cycleLiftArmToCycle(true);
                } else {
                    robot.lowerLiftArmToIntake(true);
                }
                robot.delay(0.5);
                robot.wideClaw();
                robot.delay(0.5);
                if(i!=3) {
                    robot.iterateConestackDown();
                }
                else{
                    robot.liftToPosition(0);
                }
                robot.closeClaw(false);
                robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
                robot.delay(0.0 + 0.002 * (3 - i));
                robot.liftToPosition(LIFT_MED_JUNCTION);
                robot.delay(0.42 + 0.02 * (3 - i));
                robot.raiseLiftArmToOuttake(true);
                robot.delay(0.15);
                robot.openClaw(false);
            }
//
//            robot.lowerLiftArmToIntake(false);
//            robot.delay(1);
//            robot.wideClaw();
//            robot.delay(0.5);
//            robot.liftToPosition((int) stackPos[4]);
//            robot.followTrajectorySequenceAsync(pickupTrajectory2);
//            robot.waitForFinish();
//            robot.closeClaw(false);
//            robot.waitForFinish();
//            robot.raiseLiftArmToOuttake(true);
//            robot.delay(0.3);
//            robot.liftToPosition(LIFT_HIGH_JUNCTION);
//            robot.followTrajectorySequenceAsync(dropTrajectory);
//            robot.delay(1.5);
//            robot.openClaw();
//            robot.waitForFinish();
//            robot.lowerLiftArmToIntake(true);
//            robot.delay(1);
//            robot.liftToPosition(0);
//            robot.delay(0.7);

            robot.delay(0.5);
            robot.lowerLiftArmToIntake(true);
            robot.delay(1.0);
            robot.wideClaw();
            robot.delay(1.5);
            robot.liftToPosition(0);


            if (dummyP == 1) {
                robot.followTrajectorySequenceAsync(park1trajectory);
            } else if (dummyP == 3) {
                robot.followTrajectorySequenceAsync(park3trajectory);
            } else {
                robot.followTrajectorySequenceAsync(park2trajectory);
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