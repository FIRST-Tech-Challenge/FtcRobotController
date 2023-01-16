package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "BlueRightAutoCycleTuned")


public class BlueRightAutoCycleTuned extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dropX = -30, dropY = 3, dropA = toRadians(140), dropET = toRadians(310);

    public static double pickupX1 = -45.5, pickupY1 = 11.75, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -62.9, pickupY2 = 11.75, pickupA2 = toRadians(180), pickupET2 = toRadians(180);
    double[] stackPos = {420,350,255,50,0};


    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(90)))
                .setReversed(true).splineTo(new Vector2d(-36, 40), toRadians(260))
//                .splineToSplineHeading(new Pose2d(-33, 14,toRadians(140)), toRadians(310))
                .splineToSplineHeading(new Pose2d(-29.8, 4.75, toRadians(140)), toRadians(310))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence pickupSecondPartTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-52, 11.75, toRadians(180)))
                .splineToSplineHeading(new Pose2d(-62.8, 11.75,toRadians(180)), toRadians(180))
//                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.8, 4.75, toRadians(140)))
//                .splineToSplineHeading(new Pose2d(-45, 11.75, toRadians(180)), toRadians(178))
                .splineToSplineHeading(new Pose2d(-52.4, 7,toRadians(180)), toRadians(184))
                .splineToConstantHeading(new Vector2d(-63, 11.75), toRadians(180))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence dropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-62, 12.0,toRadians(180))).setReversed(true)
//                .splineToSplineHeading(new Pose2d(-40,9.5,toRadians(140)),toRadians(330))
                .splineToSplineHeading(new Pose2d(dropX, dropY,toRadians(140)), toRadians(330))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence pickupTrajectory2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, toRadians(140)))
                .splineToSplineHeading(new Pose2d(-52.4, 7,toRadians(180)), toRadians(184))
                .splineToConstantHeading(new Vector2d(-63, 11.5), toRadians(180))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToSplineHeading(new Pose2d(-36, 33, toRadians(90)), toRadians(90))
                .build();
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToLinearHeading(new Pose2d(dropX-4,dropY+7, toRadians(185)),toRadians(dropA))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-10,12,toRadians(180)), toRadians(0))
                .build();
        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(dropX-4,dropY+7, toRadians(185)),toRadians(dropA))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToSplineHeading(new Pose2d(-60, 13, toRadians(180)), toRadians(180))
                .build();
        resetRuntime();
        robot.cp1shot();
        while(!isStarted()){
            telemetry.addData("pos",robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
            if(getRuntime()>3){
                int color = robot.cv.getPosition();
                dummyP = color;

                if (color == 1) {
                    robot.heartbeatRed();
                }
                else if (color == 2) {
                    robot.darkGreen();
                }
                else {
                    robot.violet();
                }
            }
        }
        resetRuntime();
        dummyP = robot.cv.getPosition();
        if(robot.cv.isStreaming()) {
            robot.cv.stopCamera();
        }

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested() && getRuntime() < 29.8) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(initialtrajectory);
            robot.delay(0.3);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.9);
            robot.liftToPosition(1720);
            robot.openClaw(false);
            robot.delay(0.4);
            robot.cycleLiftArmToCycle(true);
            robot.delay(0.5);
            robot.wideClaw();
            robot.delay(0.5);
            robot.liftToPosition((int) stackPos[0],true);
            robot.followTrajectorySequenceAsync(pickupTrajectory);
            robot.closeClaw(false);
            robot.followTrajectorySequenceAsync(dropTrajectory);
            robot.raiseLiftArmToOuttake(true);
            robot.liftToPosition(1720);
            robot.waitForFinish();
//            robot.liftToPosition((int) (LIFT_HIGH_JUNCTION.getValue()-100),false);
            robot.openClaw(false);
            for (int i = 0; i < 4; i++) {
                robot.delay(0.4);
                robot.cycleLiftArmToCycle(true);
                robot.delay(0.7);
                robot.wideClaw();
                robot.delay(0.7);
                robot.liftToPosition((int) stackPos[i + 1]);
                robot.followTrajectorySequenceAsync(pickupTrajectory2);
                robot.closeClaw(false);
                robot.followTrajectorySequenceAsync(dropTrajectory);
                robot.delay(0.3);
                robot.raiseLiftArmToOuttake(true);
                robot.liftToPosition(1720);
//                robot.liftToPosition((int) (LIFT_HIGH_JUNCTION.getValue()-100),false);
                robot.openClaw(false);
            }
            robot.delay(0.8);
            robot.lowerLiftArmToIntake(true);
            robot.delay(1.5);
            robot.wideClaw();
            robot.delay(1.5);
            robot.liftToPosition(0);

            if (dummyP == 1) {
                robot.followTrajectorySequenceAsync(park1trajectory);
            } else if (dummyP == 3) {
                robot.followTrajectoryAsync(park3trajectory);
            } else {
                robot.followTrajectorySequenceAsync(park2trajectory);
            }
            robot.queuedStop();
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
