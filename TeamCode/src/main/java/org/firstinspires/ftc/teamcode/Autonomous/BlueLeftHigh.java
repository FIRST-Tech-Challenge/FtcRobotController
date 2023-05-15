package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class BlueLeftHigh {
    private boolean boosted;
    PwPRobot robot;
    LinearOpMode op;
    double dummyP = 0, dropX = 28.4, dropY = 4.3;
    ;
    TrajectorySequence preloadtrajectory, pickupTrajectory, park1trajectory, park2trajectory, park3trajectory,clearLTrajectory, clearRTrajectory;
    ArrayList<TrajectorySequence> dropTrajectory, pick;
    ArrayList<Boolean> clObL , clObR;

    public BlueLeftHigh(boolean boost, LinearOpMode p_op) {
        boosted = boost;
        op = p_op;
        robot = new PwPRobot(op, false);
        op.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        Pose2d startPose = new Pose2d(36, 63.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);
        robot.cv.observeSleeve();
        op.resetRuntime();
        robot.cp1shot();
        preloadtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(36, 63.25, Math.toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(36, 40.25, toRadians(90)), toRadians(270))
                .splineTo(new Vector2d(35, 22), toRadians(265))
                .splineToSplineHeading(new Pose2d(27, 4.5, toRadians(40)), toRadians(230))
                .build();
        pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(26.5, 7.2, Math.toRadians(50)))
                .setReversed(false)
                .splineTo(new Vector2d(60.5, 11.5), Math.toRadians(0))
                .addTemporalMarker(robot::done)
                .build();

        dropTrajectory = new ArrayList<>();
        pick = new ArrayList<>();
        clObL = new ArrayList<>();
        clObR = new ArrayList<>();
        for(int i=0;i<6;i++){
            clObL.add(false);
            clObR.add(false);
        }

        for (int i = 0; i < 5; i++) {
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(64, 11.75, Math.toRadians(0)))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(dropX, dropY + 0.35 * i, Math.toRadians(35)), Math.toRadians(215))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        for (int i = 0; i < 5; i++) {
            pick.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, Math.toRadians(30)))
                    .setReversed(false)
//                    .splineToSplineHeading(new Pose2d(48, 11.75, Math.toRadians(0)), Math.toRadians(0))
                    .splineTo(new Vector2d(60.5, 11.5), Math.toRadians(0))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        clearLTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30, 9, Math.toRadians(0)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(58, 18.5,Math.toRadians(70)), Math.toRadians(70))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(56, 11.5,Math.toRadians(0)), Math.toRadians(180))
                .build();
        clearRTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30, 9, Math.toRadians(0)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(58, 2.5,Math.toRadians(70)), Math.toRadians(70))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(56, 11.5,Math.toRadians(0)), Math.toRadians(180))
                .build();
        park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30, 5, Math.toRadians(40)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(61, 12, Math.toRadians(0)), Math.toRadians(0))
                .build();
        park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30, 5, Math.toRadians(40)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(36, 17, Math.toRadians(90)), Math.toRadians(90))
                .build();
        park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30, 5, Math.toRadians(40)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35, 7), Math.toRadians(130))
                .splineTo(new Vector2d(12, 16), Math.toRadians(180))
                .build();
        while (!op.isStarted()) {
            op.telemetry.addData("pos", robot.cv.getPosition());
            op.telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
//            telemetry.addData("ANGLE:", robot.getAngleToConeStack());
            op.telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
            if (op.getRuntime() > 3) {
                dummyP = robot.cv.getPosition();

                if (dummyP == 1) {
                    robot.heartbeatRed();
                } else if (dummyP == 2) {
                    robot.darkGreen();
                } else {
                    robot.blue();
                }
            }
        }
        op.resetRuntime();
        robot.cv.observeStick();
        robot.cv.observeCone();
    }

    public void preload() {
        robot.followTrajectorySequenceAsync(preloadtrajectory);
        robot.delay(1.2);
        robot.updateTrajectoryWithCam();
        robot.delay(0.3);
        robot.raiseLiftArmToOuttake(true);
        robot.delay(0.5);
        robot.liftToPosition(LIFT_HIGH_JUNCTION);
        robot.openClaw(false);
    }

    public boolean pick(int i) {
        int temp = i;
        if (i == 0) {
            robot.followTrajectorySequenceAsync(pickupTrajectory);
        } else {
            temp = i-1;
            robot.followTrajectorySequenceAsync(pick.get(temp));
        }
        if (boosted) {
            if(robot.queuer.queue(true,!robot.roadrun.isBusy() && robot.roadrun.getCurrentTraj().end().vec().equals(pick.get(temp).end().vec()))) {
                if (robot.isObL() && !clObL.get(i) && !clObR.get(i)) {
                    clObL.set(i, true);
                }
                if(robot.isObR()&&!clObR.get(i)&&!clObL.get(i)){
                    clObR.set(i,true);
                }
            }
            clearObstacleL(i);
            clearObstacleR(i);
        }
        robot.delay(0.15);
        robot.cycleLiftArmToCycle(true);
        robot.delay(0.15);
        robot.wideClaw();
        robot.delay(0.35);
        if (i != 3) {
            robot.iterateConestackDown();
        } else {
            robot.liftToPosition(0);
            robot.lowerLiftArmToIntake();
        }
        robot.closeClaw(false);
        robot.delay(0.4);
        if (boosted && robot.queuer.queue(true, true) && !robot.clawSwitch.isSwitched()) {
            return rePick();
        }
        return true;
    }

    public void drop(int i) {
        robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
        i--;
        robot.delay(0.03 + 0.005 * (3 - i));
        robot.liftToPosition(LIFT_HIGH_JUNCTION);
        robot.delay(0.36 + 0.005 * (3 - i));
        robot.raiseLiftArmToOuttake(true);
        robot.delay(0.2);
        robot.wideClaw(false);
    }

    public void clearObstacleL(int i) {
        robot.setToNow(clObL.get(i));
        robot.followTrajectorySequenceAsync(clearLTrajectory,clObL.get(i));
        if (i == 0) {
            robot.followTrajectorySequenceAsync(pickupTrajectory,clObL.get(i));
        } else {
            i--;
            robot.followTrajectorySequenceAsync(pick.get(i),clObL.get(i));
        }
    }

    public void clearObstacleR(int i) {
        robot.setToNow(clObR.get(i));
        robot.followTrajectorySequenceAsync(clearRTrajectory,clObR.get(i));
        if (i == 0) {
            robot.followTrajectorySequenceAsync(pickupTrajectory,clObR.get(i));
        } else {
            i--;
            robot.followTrajectorySequenceAsync(pick.get(i),clObR.get(i));
        }
    }

    public boolean rePick() {
        return !robot.clawSwitch.isSwitched();
    }

    public void reDrop() {

    }

    public void park() {
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
    }

    public void update() {
        robot.setFirstLoop(false);
        robot.liftToTargetAuto();
        robot.roadrun.update();
        robot.updateClawStates();
        robot.updateLiftArmStates();
    }

    public void theWholeProgram() {
        init();
        abort:
        while (op.getRuntime() < 26 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop())) {
            preload();
            for (int i = 0; i < 5; i++) {
                if (!pick(i)) {
                    break abort;
                }
                drop(i);
            }
            update();
        }
        robot.queuer.reset();
        while (op.getRuntime() < 29.8 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop())) {
            park();
            update();
        }
        robot.stop();
    }
}
