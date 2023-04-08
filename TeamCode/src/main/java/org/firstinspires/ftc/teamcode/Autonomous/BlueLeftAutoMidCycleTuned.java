package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_MED_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
@Disabled

@Config
@Autonomous(name = "BlueLeftAutoMidCycleTuned")


public class BlueLeftAutoMidCycleTuned extends LinearOpMode {

    public static double dummyP = 3;

    public static double dropX = 30, dropY = 19.5, dropA = toRadians(330), dropET = toRadians(150);

    public static double pickupX1 = -46, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = 65, pickupY2 = 14, pickupA2 = toRadians(0), pickupET2 = toRadians(0);

    double[] stackPos = {420*1.03,330*1.03,235*1.03,80*1.03,0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(38, 63.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeMidSleeve();

        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(38, 63.25,
                        Math.toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(34, 25, toRadians(80)), toRadians(250))
                .splineToSplineHeading(new Pose2d(33, 9, toRadians(315)), toRadians(270))
                .lineToLinearHeading(new Pose2d(31,19.25, toRadians(315)))
                .addTemporalMarker(robot::done)
                .build();

        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,19.25, toRadians(315)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(44,pickupY2 - 0.5,pickupA2),pickupET2)
                .splineToSplineHeading(new Pose2d(pickupX2,pickupY2-0.5,pickupA2), pickupET2)
//                .splineToSplineHeading()
//                .splineToSplineHeading(new Pose2d(pickupX2-1,pickupY2-0.5,pickupA2),pickupET2)
//                .addTemporalMarker(()->{robot.done(); robot.roadrun.breakFollowing();})
                .addTemporalMarker(robot::done)
                .build();
        ArrayList<TrajectorySequence> dropTrajectory = new ArrayList<>();
        for(int i=0;i<5;i++) {
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2,
                            pickupY2 - 0.5 + 0.075 * i, pickupA2))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(dropX, dropY + 0.5, dropA), dropET)
                    .addTemporalMarker(robot::done)
                    .build());
        }
        ArrayList<TrajectorySequence> pickupTrajectory2 = new ArrayList<>();
        for(int i=0;i<5;i++) {
            pickupTrajectory2.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX + 1, dropY - 0.5, dropA))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(50, pickupY2 - 0.5 + 0.075 * i, pickupA2), pickupET2)
                .splineToSplineHeading(new Pose2d(pickupX2, pickupY2 - 0.25 + 0.075 * i, pickupA2), pickupET2)
                .addTemporalMarker(robot::done)
                .build());
        }

//        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(42.85, 63.25, Math.toRadians(90)))
//                .setReversed(true)
////                .splineTo(new Vector2d(34,50),toRadians(270))
////                .lineTo(new Vector2d(37.5,56))
////                .splineToSplineHeading(new Pose2d(32, 30.5, toRadians(45)), toRadians(270))
//                .splineToSplineHeading(new Pose2d(38, 51, toRadians(70)), toRadians(250))
//                .addDisplacementMarker(10,()->{robot.roadrun.followTrajectorySequenceAsync(initialTrajectoryPart2);})
//                .splineToSplineHeading(new Pose2d(36, 17, toRadians(90)), toRadians(270))
//                .splineToSplineHeading(new Pose2d(29,7, toRadians(45)), toRadians(215))
//                .build();
//        TrajectorySequence pickupTrajectoryPart2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2-15,16,0))
//                .setReversed(false)
////                                        .splineToLinearHeading(new Pose2d(36,11.5, toRadians(0)),toRadians(240))
//                .splineToSplineHeading(new Pose2d(pickupX2, pickupY2,toRadians(0)), toRadians(0))
//                .addTemporalMarker(robot::done)
//                .build();


//        TrajectorySequence drop1Trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2, pickupY2, 0))
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(dropX,dropY, dropA), dropET)
//                .build();
//        TrajectorySequence pickupTrajectory2Part2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2-15,15,0))
//                .setReversed(false)
////                                        .splineToLinearHeading(new Pose2d(36,11.5, toRadians(0)),toRadians(240))
//                .splineToSplineHeading(new Pose2d(pickupX2, pickupY2,toRadians(0)), toRadians(0))
//                .addTemporalMarker(robot::done)
//                .build();

//        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
//                .splineToSplineHeading(new Pose2d(-36, 33, toRadians(90)), toRadians(90))
//                .build();
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(60,16, toRadians(0)),toRadians(90))
                .splineToLinearHeading(new Pose2d(60,20, toRadians(90)),toRadians(90))
                .build();

        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .lineToLinearHeading(new Pose2d(38, 10,dropA))
                .lineToLinearHeading(new Pose2d(38, 18,toRadians(90)))
                .build();
        TrajectorySequence park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY, dropA))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(dropX+4,dropY-9, toRadians(-5)),toRadians(dropA))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(13,12,toRadians(0)), toRadians(0))
                .splineToLinearHeading(new Pose2d(13,16,toRadians(90)), toRadians(0))
                .build();

        while(!isStarted()){
            telemetry.addData("pos",robot.cv.getMidPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
            if(getRuntime()>3){
                int color = robot.cv.getMidPosition();

                if (color == 1) {
                    robot.heartbeatRed();
                }
                else if (color == 2) {
                    robot.darkGreen();
                }
                else {
                    robot.blue();
                }
            }
        }
        resetRuntime();
        dummyP = robot.cv.getMidPosition();

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested()) {
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
            robot.liftToPosition((int) stackPos[0]);
            robot.followTrajectorySequenceAsync(pickupTrajectory);
            robot.closeClaw(false);
            robot.followTrajectorySequenceAsync(dropTrajectory.get(0));
            robot.liftToPosition((int)LIFT_MED_JUNCTION.getValue(), true);
            robot.delay(0.55);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.25);
            robot.openClaw(false);
            for (int i = 0; i < 4; i++) {
                robot.followTrajectorySequenceAsync(pickupTrajectory2.get(i));
                if(i!=3) {
                    robot.cycleLiftArmToCycle(true);
                }else{
                    robot.lowerLiftArmToIntake(true);
                }
                robot.delay(0.5);
                robot.wideClaw();
                robot.delay(0.5);
                robot.liftToPosition((int) stackPos[i + 1]);
                robot.closeClaw(false);
                robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
                robot.delay(0.1+0.005*(3-i));
                robot.liftToPosition(LIFT_MED_JUNCTION);
                robot.delay(0.36+0.005*(3-i));
                robot.raiseLiftArmToOuttake(true);
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
            robot.delay(1.6);
            robot.wideClaw();
            robot.delay(2.0);
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
