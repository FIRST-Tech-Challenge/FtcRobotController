package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive.getVelocityConstraint;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class BlueLeftHigh {
    private boolean boosted, tooHot = false, lowBattery = false;
    private PwPRobot robot = null;
    private double sleeveP = 0, lastTime = 0;
    private final double DROP_X = 28.0, DROP_Y = 4.7;
    private TrajectorySequence preloadtrajectory = null, pickupTrajectory = null, park1trajectory = null,
            park2trajectory = null, park3trajectory = null, reDropTrajectory = null;
    private ArrayList<TrajectorySequence> dropTrajectory = null, pick = null;
    private ArrayList<Boolean> clObL = null, clObR = null, pInView = null;

    public BlueLeftHigh(boolean boost, LinearOpMode p_op, PwPRobot p_robot) {
        //store parameters
        boosted = boost;
        op = p_op;
        robot = p_robot;
        op.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void init() {
        //set startPose
        Pose2d startPose = new Pose2d(36, 63.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);
        //turn on sleeve camera pipeline
        robot.cv.observeSleeve();
        //reset run time
        op.resetRuntime();
        //set loading screen LED lights
        robot.cp1shot();
        //initialize and build trajectories
        preloadtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(36, 63.25, Math.toRadians(90)))
                .addTemporalMarker(() -> robot.setPoling(true))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(36, 36.25, toRadians(90)), toRadians(270))
                .splineTo(new Vector2d(27.0, 6.2), toRadians(220))
                .build();
        pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(28.7, 6.7, Math.toRadians(37)))
                .setReversed(false)
                .splineTo(new Vector2d(63.0, 11.75), Math.toRadians(0))
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
            pick.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(DROP_X, DROP_Y, Math.toRadians(35)))
                    .setReversed(false)
                    .splineTo(new Vector2d(63-0.1*i, 11.75+0.4*i), Math.toRadians(0))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        for (int i = 0; i < 5; i++) {
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(60.5,11.75,Math.toRadians(0)))
                    .addTemporalMarker(0, () -> robot.setPoling(true))
                    .setReversed(true)
                    .splineTo(new Vector2d(DROP_X, DROP_Y), Math.toRadians(215),
                            getVelocityConstraint(70,9,17), getAccelerationConstraint(47))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        reDropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(DROP_X, DROP_Y, Math.toRadians(35)))
                .addTemporalMarker(() -> robot.setPoling(true))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(DROP_X + 6.4, DROP_Y + 3.2, Math.toRadians(35)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(DROP_X, DROP_Y, Math.toRadians(37)))
                .addTemporalMarker(robot::done)
                .build();

        park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(DROP_X, DROP_Y, Math.toRadians(35)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(60.5, 12, Math.toRadians(0)), Math.toRadians(0))
                .build();
        park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(DROP_X, DROP_Y, Math.toRadians(35)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(35.5, 17, Math.toRadians(90)), Math.toRadians(90))
                .build();
        park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(DROP_X, DROP_Y, Math.toRadians(35)))
                .lineToLinearHeading(new Pose2d(DROP_X + 6.23, DROP_Y + 8.1, toRadians(0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(9.87, 13.24, toRadians(0)), toRadians(0))
                .build();
        //waitForStart and record sleeve position
        while (!op.isStarted() && !op.isStopRequested()) {
            sleeveP = robot.checkRobotReadiness();
        }
        //reset runtime
        op.resetRuntime();
        robot.updateTime();
        //switch to pole pipeline
        robot.cv.observeStick();
        robot.setFirstLoop(true);
        lastTime = 1;
    }

    public void preload() {
        robot.followTrajectorySequenceAsync(preloadtrajectory);
        //update with camera pole position if boosted
        if (boosted) {
            robot.delay(1.0);
            robot.updateTrajectoryWithCam();
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
        //if boosted check distance sensors for obstacles, if obstacles then clear obstacle right or left
        if (boosted) {
            boolean[] vals = robot.checkIsOb(clObL.get(i), clObR.get(i), pick.get(0).end());
            clObL.set(i, vals[0]);
            clObR.set(i, vals[1]);
            robot.delay(0.2);
            clearObstacleL(i);
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
        //abort if boosted and clawSwitch not as expected
        if (boosted && robot.queuer.queue(true, robot.clawSwitch.isSwitched()) && !robot.clawSwitch.isSwitched()) {
            return rePick();
        }
        return true;
    }

    public void drop(int i) {
        robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
        //update with camera pole position if boosted
        if (boosted) {
            robot.delay(0.2);
            robot.updateTrajectoryWithCam();
        }
        robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue()-10);
        robot.delay(0.1);
        robot.raiseLiftArmToOuttake(true);
        if (boosted) {
            robot.delay(0.1);
            boolean val = robot.poleInView(pInView.get(i + 1));
            pInView.set(i + 1, val);
            reDrop(i + 1);
        }
        robot.wideClaw(false);
    }

    public void reDrop(int i) {
        robot.followTrajectorySequenceAsync(reDropTrajectory, pInView.get(i));
    }

    public void clearObstacleL(int i) {
        robot.delay(0.2);
        robot.clearObstacle(new Pose2d(59, 20, Math.toRadians(90)), clObL.get(i) || clObR.get(i));

    }

    public boolean rePick() {
        return robot.clawSwitch.isSwitched();
    }

    public void park() {
        if (sleeveP == 1) {
            robot.followTrajectorySequenceAsync(park1trajectory, 1);
        } else if (sleeveP == 3) {
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
}
