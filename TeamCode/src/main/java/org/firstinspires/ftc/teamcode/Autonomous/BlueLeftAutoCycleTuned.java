package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
//@Disabled

@Config
@Autonomous(name = "BlueLeftAutoCycleTuned")


public class BlueLeftAutoCycleTuned extends LinearOpMode {

    public static double dummyP = 2;

    public static double dropX=28.4, dropY=4.3;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(36, 63.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);


        robot.cv.observeSleeve();

        TrajectorySequence preloadtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(36,63.25, Math.toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(36, 40.25, toRadians(90)), toRadians(270))
                .splineTo(new Vector2d(35, 22), toRadians(265))
                .splineToSplineHeading(new Pose2d(27, 4.5, toRadians(40)), toRadians(230))
                .build();

        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(27,4.5,Math.toRadians(40)))
                .setReversed(false)
                .splineTo(new Vector2d(63.5, 11.5), Math.toRadians(0))
                .addTemporalMarker(robot::done)
                .build();

        ArrayList<TrajectorySequence> dropTrajectory = new ArrayList<>();
        ArrayList<TrajectorySequence> pick = new ArrayList<>();

        for(int i=0;i<5;i++){
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(63.5,11.5 + 0.35*i,Math.toRadians(0)))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(dropX, dropY+0.35*i, Math.toRadians(35)), Math.toRadians(215))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        for(int i=0;i<5;i++){
            pick.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY + 0.35*i,Math.toRadians(35)))
                    .setReversed(false)
                    .splineTo(new Vector2d(63.5, 10.5+0.35*i), Math.toRadians(0))
                    .addTemporalMarker(robot::done)
                    .build());
        }

        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(35)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(61, 12, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(35)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(36, 17, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(35)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35, 7), Math.toRadians(130))
                .splineTo(new Vector2d(12, 16), Math.toRadians(180))
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
                dummyP = robot.cv.getPosition();

                if ( dummyP== 1) {
                    robot.heartbeatRed();
                }
                else if (dummyP== 2) {
                    robot.darkGreen();
                }
                else {
                    robot.blue();
                }
            }
        }
        resetRuntime();
        robot.cv.observeStick();
        waitForStart();
        if (isStopRequested()) return;

        logger.log("/RobotLogs/GeneralRobot", "POSITION:" + robot.roadrun.getPoseEstimate(), false);
        while (opModeIsActive() && !isStopRequested() && getRuntime()<29.8) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(preloadtrajectory);
            robot.delay(1.2);
            robot.updateTrajectoryWithCam();
            robot.delay(0.3);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.5);
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.openClaw(  false);
            robot.delay(0.4);
            robot.cycleLiftArmToCycle(true);
            robot.delay(0.5);
            robot.wideClaw();
            robot.delay(0.5);
            robot.iterateConestackUp();
            robot.followTrajectorySequenceAsync(pickupTrajectory);
            robot.closeClaw(false);
            robot.followTrajectorySequenceAsync(dropTrajectory.get(0));
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.delay(0.55);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.2);
            robot.wideClaw(false);
            robot.delay(0.25);
            for (int i = 0; i < 4; i++) {
                robot.followTrajectorySequenceAsync(pick.get(i));
                robot.delay(0.1);
                robot.cycleLiftArmToCycle(true);
                robot.delay(0.1);
                robot.wideClaw();
                if(i<3) {
                    robot.delay(0.35);

                    robot.iterateConestackDown();
                }
                if(i==3){
                    robot.delay(0.3);

                    robot.liftToPosition(0);
                }

                robot.closeClaw(false);
                robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
                robot.delay(0.3);
//                robot.updateTrajectoryWithCam();
                robot.delay(0.03+0.005*(3-i));
                robot.liftToPosition(LIFT_HIGH_JUNCTION);
                robot.delay(0.36+0.005*(3-i));
                robot.raiseLiftArmToOuttake(true);
                robot.delay(0.2);
                robot.wideClaw(false);
                robot.delay(0.15);
            }

            if (dummyP == 1) {
                robot.followTrajectorySequenceAsync(park1trajectory);
            } else if (dummyP == 3) {
                robot.followTrajectorySequenceAsync(park3trajectory);
            } else {
                robot.followTrajectorySequenceAsync(park2trajectory);
            }

            robot.delay(0.3);

            robot.lowerLiftArmToIntake(true);
            robot.delay(1.6);
            robot.wideClaw();
            robot.delay(0.8);
            robot.liftToPosition(0);


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
