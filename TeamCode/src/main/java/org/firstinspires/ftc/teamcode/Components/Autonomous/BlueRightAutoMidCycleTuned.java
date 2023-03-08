package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_MED_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Config
@Autonomous(name = "BlueRightAutoMidCycleTuned")


public class BlueRightAutoMidCycleTuned extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dropX = -29, dropY = 18, dropA = toRadians(210), dropET = toRadians(30);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -63, pickupY2 = 13, pickupA2 = toRadians(180), pickupET2 = toRadians(180);

    double[] stackPos = {440, 330, 245, 100, 0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-30.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.6, 63.25, toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-32, 52, toRadians(70)), toRadians(250))
                .splineToSplineHeading(new Pose2d(-34, 9, toRadians(225)), toRadians(270))
                .lineToLinearHeading(new Pose2d(-29, 18.5, toRadians(225)))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29, 18.5, toRadians(225)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(pickupX2 + 5 - 0.5, pickupY2 - 0.5, toRadians(180)), toRadians(180))
//                .splineToSplineHeading()
                .splineToSplineHeading(new Pose2d(pickupX2 - 0.75, pickupY2 - 1.25, pickupA2), pickupET2)
//                .addTemporalMarker(()->{robot.done(); robot.roadrun.breakFollowing();})
                .addTemporalMarker(robot::done)
                .build();
//        TrajectorySequence dropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2, pickupY2, pickupA2))
//                .setReversed(true)
////                .splineToSplineHeading(new Pose2d(pickupX2+15, pickupY2,pickupA2),0)
//                .splineToSplineHeading(new Pose2d(dropX,dropY,dropA),dropET)
//                .UNSTABLE_addTemporalMarkerOffset(0.4,robot::done)
//                .build();
        ArrayList<TrajectorySequence> dropTrajectory = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2 - 1, pickupY2, pickupA2))
                    .setTangentOffset((toRadians(180)))
                    .splineToSplineHeading(new Pose2d(dropX/*+i*1.3*/, dropY/*-1.2*i*/, dropA), dropET)
//                    .UNSTABLE_addTemporalMarkerOffset(0.4,robot::done)
                    .addTemporalMarker(robot::done)
                    .build());
        }

        ArrayList<TrajectorySequence> pickupTrajectory2 = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            pickupTrajectory2.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX , dropY - 1, dropA))
                    .setReversed(false)
                    .splineToSplineHeading(new Pose2d(pickupX2 -0.8 , pickupY2 , pickupA2 ), pickupET2)
//                .addTemporalMarker(()->{robot.done(); robot.roadrun.breakFollowing();})
                    .addTemporalMarker(robot::done)
                    .build());
        }
        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToSplineHeading(new Pose2d(-36, 33, toRadians(90)), toRadians(90))
                .build();
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .setReversed(false)
//                .splineToSplineHeading(new Pose2d(-50, 10, toRadians(180)), toRadians(180))
//                .setReversed(true)
//                .lineTo(new Vector2d(-10, 9))
                .lineToConstantHeading(new Vector2d(dropX-2,dropY-2))
                .splineToLinearHeading(new Pose2d(-12, 12, toRadians(270)), toRadians(135))
                .build();
        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .lineToConstantHeading(new Vector2d(dropX-5,dropY-5))
//                .splineToConstantHeading(new Vector2d(dropX-3, dropY-3),toRadians(225))
                .lineToLinearHeading(new Pose2d(-33, 12, toRadians(90)))
                .build();
        TrajectorySequence park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
//                .lineToConstantHeading(new Vector2d(dropX-3,dropY-3))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-59, 14, toRadians(180)), toRadians(180))
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
        robot.cv.observeStick();

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
            robot.delay(0.3);
            robot.updateTrajectoryWithCam();
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
                robot.delay(0.3);
                robot.updateTrajectoryWithCam();
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

            robot.delay(0.9);
            robot.lowerLiftArmToIntake(true);
            robot.delay(1.0);
            robot.wideClaw();
            robot.delay(1.0);
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
