package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import static java.lang.Math.toRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class BlueLeftHigh {
    private boolean boosted, tooHot = false, lowBattery = false;
    PwPRobot robot = null;
    LinearOpMode op;
    double dummyP = 0, dropX = 31.4, dropY = 1.6, lastTime = 0, thisTime = 0, startTime = 0, voltage = 0;
    ;
    TrajectorySequence preloadtrajectory = null, pickupTrajectory = null, park1trajectory = null,
            park2trajectory = null, park3trajectory = null, clearLTrajectory = null, clearRTrajectory = null,
            closeLTrajectory = null, closeRTrajectory = null, reDropTrajectory = null;
    ArrayList<TrajectorySequence> dropTrajectory = null, pick = null;
    ArrayList<Boolean> clObL = null, clObR = null, pInView = null;

    public BlueLeftHigh(boolean boost, LinearOpMode p_op, PwPRobot p_robot) {
        boosted = boost;
        op = p_op;
        robot = p_robot;
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
                .addTemporalMarker(() -> robot.setPoling(true))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(36, 40.25, toRadians(90)), toRadians(270))
//                .splineTo(new Vector2d(34, 28), toRadians(245))
                .splineToSplineHeading(new Pose2d(31.2, 4.2, Math.toRadians(40)), toRadians(220),
                        getVelocityConstraint(110, 9, 14), getAccelerationConstraint(60))
                .build();

        pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(31.5, 2.5, Math.toRadians(40)))
                .addTemporalMarker(() -> robot.setConing(true))
                .setReversed(false)
//                .splineTo(new Vector2d(51,11.5),Math.toRadians(0))
                .splineTo(new Vector2d(66, 10.75), Math.toRadians(0))
                .addTemporalMarker(robot::done)
                .build();

        dropTrajectory = new ArrayList<>();
        pick = new ArrayList<>();
        clObL = new ArrayList<>();
        clObR = new ArrayList<>();

        pInView = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            clObL.add(false);
            clObR.add(false);
            pInView.add(false);
        }

        for (int i = 0; i < 5; i++) {
            pick.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY-0.4*i, Math.toRadians(37)))
                    .addTemporalMarker(() -> robot.setConing(true))
                    .setReversed(false)
//                    .splineTo(new Vector2d(51,11.5),Math.toRadians(0))
                    .splineTo(new Vector2d(66.3, 10.75), Math.toRadians(0))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        for (int i = 0; i < 5; i++) {
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(pick.get(i).end())
                    .addTemporalMarker(0, () -> robot.setPoling(true))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(dropX-0.3*i, dropY-0.1*i, Math.toRadians(37)), Math.toRadians(217))
//                            getVelocityConstraint(110,9,14), getAccelerationConstraint(51))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        reDropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, Math.toRadians(37)))
                .addTemporalMarker(() -> robot.setPoling(true))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(dropX + 6.3, dropY + 3.2, Math.toRadians(37)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(dropX + 1, dropY + 1, Math.toRadians(37)))
                .addTemporalMarker(robot::done)
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
                .splineTo(new Vector2d(dropX + 6, dropY + 9), toRadians(5))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(14, 13, toRadians(0)), toRadians(0))
                .build();
        while (!op.isStarted() && !op.isStopRequested()) {
            dummyP = robot.checkRobotReadiness();
        }
        op.resetRuntime();
        robot.updateTime();
        robot.cv.observeStick();
        robot.setFirstLoop(true);
        lastTime = 1;
//        robot.cv.observeCone();
    }

    public void preload() {
        robot.followTrajectorySequenceAsync(preloadtrajectory);
        if (boosted) {
//            robot.delay(1.0);
//            robot.updateTrajectoryWithCam();
        }
        robot.delay(0.3);
        robot.raiseLiftArmToOuttake(true);
        robot.delay(0.5);
        robot.liftToPosition(LIFT_HIGH_JUNCTION);
        robot.wideClaw(false);
    }

    public boolean pick(int i) {
        int temp = i - 1;
        if (i == 0) {
            robot.followTrajectorySequenceAsync(pickupTrajectory);
        } else {
            robot.followTrajectorySequenceAsync(pick.get(temp));
        }
        if (boosted) {
            boolean[] vals = robot.checkIsOb(clObL.get(i), clObR.get(i), pick.get(0).end());
            clObL.set(i, vals[0]);
            clObR.set(i, vals[1]);
//            robot.delay(0.2);
//            robot.updateTrajectoryWithCone();
            robot.delay(0.2);
            clearObstacleL(i);
//            clearObstacleR(i);
        }
        robot.delay(0.1);
        robot.cycleLiftArmToCycle(true);
        robot.delay(0.1);
        robot.wideClaw();
        robot.delay(0.3);
        if (i == 0) {
            robot.iterateConestackUp();
        } else if (i != 4) {
            robot.iterateConestackDown();
        } else {
            robot.liftToPosition(-40);
            robot.lowerLiftArmToIntake();
        }
        robot.closeClaw(false);
        if (boosted) {
            robot.delay(0.7);
        }
        if (boosted && robot.queuer.queue(true, true) && !robot.clawSwitch.isSwitched()) {
            return rePick();
        }
        return true;
    }

    public void drop(int i) {
        robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
        if (boosted) {
            robot.delay(0.2);
            robot.updateTrajectoryWithCam();
        }
//        robot.delay(0.023 + 0.005 * (3 - i));
        robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue()-20);
        robot.delay(0.5);
        robot.raiseLiftArmToOuttake(true);
//        robot.delay(0.4);
//        robot.closeClaw(true);
        if (boosted) {
            robot.delay(0.2);
            boolean val = robot.poleInView(pInView.get(i + 1));
            pInView.set(i + 1, val);
            reDrop(i + 1);
        }
        robot.delay(0.2);
        robot.wideClaw(false);
    }

    public void reDrop(int i) {
        robot.followTrajectorySequenceAsync(reDropTrajectory, pInView.get(i));
    }

    public void clearObstacleL(int i) {
//        robot.setToNow(clObL.get(i));
        robot.delay(0.2);
        robot.clearObstacle(new Pose2d(59, 20, Math.toRadians(90)), clObL.get(i) || clObR.get(i));
//            robot.setStackHeight(i, clObL.get(i)||clObR.get(i));

//        if (robot.roadrun.getPoseEstimate().vec().distTo(pickupTrajectory.end().vec()) < 10) {
//            robot.changeTrajectorySequence(closeLTrajectory, clObL.get(i));
//        } else {
//            robot.changeTrajectorySequence(clearLTrajectory, clObL.get(i));
//        }
//        if (i == 0) {
//            robot.followTrajectorySequenceAsync(pickupTrajectory,clObL.get(i));
//        } else {
//            i--;
//            robot.followTrajectorySequenceAsync(pick.get(i),clObL.get(i));
//        }
    }

//    public void clearObstacleR(int i) {
//        robot.clearObstacle(new Pose2d(55, 19.5, Math.toRadians(90)),clObR.get(i));
//        robot.setStackHeight(i,clObR.get(i));
////        robot.done();
////        robot.setToNow(clObR.get(i));
////        if (robot.roadrun.getPoseEstimate().vec().distTo(pickupTrajectory.end().vec()) < 10) {
////            robot.changeTrajectorySequence(closeRTrajectory, clObR.get(i));
////        } else {
////            robot.changeTrajectorySequence(clearRTrajectory, clObR.get(i));
////        }        if (i == 0) {
////            robot.followTrajectorySequenceAsync(pickupTrajectory,clObR.get(i));
////        } else {
////            i--;
////            robot.followTrajectorySequenceAsync(pick.get(i),clObR.get(i));
////        }
//    }

    public boolean rePick() {
        return robot.clawSwitch.isSwitched();
    }

    public void reDrop() {

    }

    public void park() {
        if (dummyP == 1) {
            robot.followTrajectorySequenceAsync(park1trajectory, 1);
        } else if (dummyP == 3) {
            robot.followTrajectorySequenceAsync(park3trajectory, 1);
        } else {
            robot.followTrajectorySequenceAsync(park2trajectory, 1);
        }
        robot.lowerLiftArmToIntake(true);
        robot.delay(2.5);
        robot.wideClaw(true);
        robot.delay(0.3);
        robot.liftToPosition(-10, true);
    }

    public void update() {
        double loopTime = time - lastTime;
        if (time > 4 && !robot.queuer.isFirstLoop() && loopTime > 0.4) {
            robot.setDistBroke(true);
        }
        lastTime = time;
        robot.setFirstLoop(false);
        robot.liftToTargetAuto();
        robot.roadrun.update();
        robot.updateClawStates();
        robot.updateLiftArmStates();
        robot.updateCV();
    }

    public void theWholeProgram() {
        init();
        abort:
        while ((time < 26.5 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop())) && !op.isStopRequested()) {
            preload();
            for (int i = 0; i < 5; i++) {
                if (!pick(i)) {
                    break abort;
                }
                drop(i);
            }
            update();
        }
        robot.done();
        robot.queuer.reset();
        robot.done();
        robot.stop();
    }
}
