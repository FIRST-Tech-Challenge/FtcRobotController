package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_MED_JUNCTION;
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

public class BlueRightMid {
    private boolean boosted,tooHot=false;
    PwPRobot robot=null;
    LinearOpMode op;
    double dummyP = 0, dropX=-29.5, dropY=19.5, dropA = toRadians(215),lastTime = 0.0,startTime=0.0;

    TrajectorySequence preloadtrajectory=null, pickupTrajectory=null, park1trajectory=null,
            park2trajectory=null, park3trajectory=null, clearLTrajectory=null, clearRTrajectory=null,
            closeLTrajectory=null, closeRTrajectory=null, reDropTrajectory=null;
    ArrayList<TrajectorySequence> dropTrajectory=null, pick=null;
    ArrayList<Boolean> clObL=null, clObR=null,pInView=null;

    public BlueRightMid(boolean boost, LinearOpMode p_op, PwPRobot p_robot) {
        boosted = boost;
        op = p_op;
        robot = p_robot;
        op.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        Pose2d startPose = new Pose2d(-35.9, 63.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);
        robot.cv.observeSleeve();
        op.resetRuntime();
        robot.cp1shot();
        preloadtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-34.5,63.25, Math.toRadians(90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-33, 46, toRadians(90)))
                .splineToSplineHeading(new Pose2d(-35, 9, toRadians(225)), toRadians(270))
                .lineToLinearHeading(new Pose2d(-28, 19, toRadians(225)))
                .addTemporalMarker(robot::done)
                .build();

//        pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-27,4.5,Math.toRadians(140)))
//                .setReversed(false)
//                .splineTo(new Vector2d(-64, 12), Math.toRadians(180))
//                .addTemporalMarker(robot::done)
//                .build();

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
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(-64.69,11,Math.toRadians(180)))
                    .setTangentOffset((toRadians(180)))
                    .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), toRadians(30))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        for (int i = 0; i < 5; i++) {
            pick.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY,dropA))
                    .setReversed(false)
                    .splineTo(new Vector2d(-64, 11.8+0.8*i), toRadians(180))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        reDropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY,Math.toRadians(149)))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(dropX+3.3,dropY+1.2,Math.toRadians(145)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(dropX,dropY,Math.toRadians(149)))
                .addTemporalMarker(robot::done)
                .build();

        park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(dropX-2,dropY-2))
                .splineToLinearHeading(new Pose2d(-9, 12, toRadians(270)), toRadians(135))
                .build();
        park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .lineToConstantHeading(new Vector2d(dropX-5,dropY-5))
                .lineToLinearHeading(new Pose2d(-33, 12, toRadians(90)))
                .build();
        park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .setReversed(false)
                .splineTo(new Vector2d(-59, 14), toRadians(180))
                .build();

        double maxLoopTime = 2.0;
        while (!op.isStarted()&&!op.isStopRequested()) {
            if(!tooHot) {
                op.telemetry.addData("pos", robot.cv.getPosition());
                op.telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
//            telemetry.addData("ANGLE:", robot.getAngleToConeStack());
                op.telemetry.update();
                robot.updateClawStates();
                robot.updateLiftArmStates();
                if (time > 3) {
                    dummyP = robot.cv.getPosition();

                    if (dummyP == 1) {
                        robot.heartbeatRed();
                    } else if (dummyP == 2) {
                        robot.darkGreen();
                    } else {
                        robot.blue();
                    }
                }
                robot.updateTime();
                if (time > 5 && startTime == 0) {
                    startTime = time;
                    for (int i = 0; i < 100; i++) {
                        update();
                    }
                    startTime = time - startTime;
                    if(startTime>maxLoopTime){
                        tooHot=true;
                    }
                }
                op.telemetry.addData("startTime", startTime);
            }
            else{
                robot.partycolorwave();
                op.telemetry.addData("TOO HOT", startTime);
                op.telemetry.update();
            }
        }
        if(op.isStopRequested()){
            robot.stop();
        }
        op.resetRuntime();
        robot.cv.observeStick();
        robot.cv.observeCone();
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
        robot.liftToPosition(LIFT_MED_JUNCTION);
        robot.wideClaw(false);
    }

    public boolean pick(int i) {
        int temp = i-1;
        if (i == 0) {
            robot.followTrajectorySequenceAsync(pick.get(0));
        } else {
            robot.followTrajectorySequenceAsync(pick.get(temp));
        }

        if (boosted) {
            boolean[] vals = robot.checkIsOb(clObL.get(i), clObR.get(i),pick.get(0).end());
            clObL.set(i, vals[0]);
            clObR.set(i, vals[1]);
//            robot.delay(0.5);
//            robot.updateTrajectoryWithCone();
            robot.delay(0.4);
            clearObstacleL(i);
//            clearObstacleR(i);
        }
        robot.delay(0.1);
        robot.cycleLiftArmToCycle(true);
        robot.delay(0.1);
        robot.wideClaw();
        robot.delay(0.3);
        if(i==0){
            robot.iterateConestackUp();
        }
        else if (i != 4) {
            robot.iterateConestackDown();
        } else {
            robot.liftToPosition(0);
            robot.lowerLiftArmToIntake();
        }
        robot.closeClaw(false);
        if(boosted) {
            robot.delay(0.5);
        }
        if (boosted && robot.queuer.queue(true, true) && !robot.clawSwitch.isSwitched()) {
            return robot.clawSwitch.isSwitched();
        }
        return true;
    }

    public void drop(int i) {
        robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
        if (boosted) {
//            robot.delay(0.4);
//            robot.updateTrajectoryWithCam();
        }
//        robot.delay(0.023 + 0.005 * (3 - i));
        robot.liftToPosition(LIFT_MED_JUNCTION);
        robot.delay(0.1 + 0.005 * (3 - i));
        robot.raiseLiftArmToOuttake(true);
        if (boosted) {
//            robot.delay(0.1);
//            boolean val = robot.poleInView(pInView.get(i+1));
//            pInView.set(i+1, val);
//            reDrop(i+1);
        }
        robot.delay(0.1);
        robot.wideClaw(false);
    }
    public void reDrop(int i){
//        robot.followTrajectorySequenceAsync(reDropTrajectory,pInView.get(i));
    }

    public void clearObstacleL(int i) {
//        robot.setToNow(clObL.get(i));
        robot.delay(0.3);
        robot.clearObstacleRight(new Pose2d(-56, 20, Math.toRadians(90)),clObL.get(i)||clObR.get(i));
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
            robot.followTrajectorySequenceAsync(park1trajectory);
        } else if (dummyP == 3) {
            robot.followTrajectorySequenceAsync(park3trajectory);
        } else {
            robot.followTrajectorySequenceAsync(park2trajectory);
        }
        robot.delay(2.0);
        robot.wideClaw(true);
        robot.delay(0.7);
        robot.liftToPosition(0,true);
        robot.delay(0.3);
        robot.lowerLiftArmToIntake(true);

    }

    public void update() {
        double loopTime = time - lastTime;
        if(!robot.queuer.isFirstLoop()&&loopTime>0.1){
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
        while ((time< 26.5 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!op.isStopRequested()) {
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
        while ((time < 29.8 && (!robot.queuer.isFullfilled() || robot.queuer.isFirstLoop()))&&!op.isStopRequested()) {
            park();
            update();
        }
        robot.stop();
    }
}
