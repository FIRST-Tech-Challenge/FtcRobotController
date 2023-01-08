package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
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

    public static double dropX = -31, dropY = 3.2, dropA = toRadians(140), dropET = toRadians(320);

    public static double pickupX1 = -45.5, pickupY1 = 10, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -62.8, pickupY2 = 10.5, pickupA2 = toRadians(180), pickupET2 = toRadians(180);
    double[] stackPos = {400, 290, 200, 80, 0};


    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(90)))
                .setReversed(true).splineToSplineHeading(new Pose2d(-36, 40, toRadians(70)), toRadians(250))
                .splineToSplineHeading(new Pose2d(-36, 26, toRadians(105)), toRadians(285))
                .splineToSplineHeading(new Pose2d(-27.5, 6.4, toRadians(120)), toRadians(290))
                .build();
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-27.5, 7.4, toRadians(120)))
                .splineToSplineHeading(new Pose2d(pickupX1, pickupY1, pickupA1), pickupET1)
                .splineToSplineHeading(new Pose2d(pickupX2, pickupY2, pickupA2), pickupET2)
                .build();
        TrajectorySequence dropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(pickupX2, pickupY2, pickupA2)).setReversed(true)
                .splineToSplineHeading(new Pose2d(dropX, dropY, dropA), dropET)
                .build();
        TrajectorySequence pickupTrajectory2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
//                .splineToSplineHeading(new Pose2d(-45.5, 10, toRadians(180)), toRadians(180))
                .splineToSplineHeading(new Pose2d(-62.8, 10.5, toRadians(180)), toRadians(180))
                .build();
        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToSplineHeading(new Pose2d(-36, 33, toRadians(90)), toRadians(90))
                .build();
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35,7), toRadians(50))
                .splineTo(new Vector2d(-10,12), toRadians(0))
                .build();

        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToLinearHeading(new Pose2d(-35.5,11,toRadians(180)), toRadians(90))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToSplineHeading(new Pose2d(-60, 13, toRadians(180)), toRadians(180))
                .build();
        while (!isStarted()) {
            telemetry.addData("pos", robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateLiftArmStates();

        }
        resetRuntime();
        dummyP = robot.cv.getPosition();
        robot.cv.stopCamera();

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested() && getRuntime() < 29.8) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(initialtrajectory);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.7);
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.waitForFinish();
            robot.openClaw(false);
            robot.cycleLiftArmToCycle(true);
            robot.delay(1.0);
            robot.wideClaw();
            robot.liftToPosition((int) stackPos[0]);
            robot.followTrajectorySequenceAsync(pickupTrajectory);
            robot.waitForFinish();
            robot.closeClaw(false);
            robot.waitForFinish();
            robot.raiseLiftArmToOuttake(true);
            robot.followTrajectorySequenceAsync(dropTrajectory);
            robot.delay(0.5);
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.waitForFinish();
            robot.liftToPosition((int) (LIFT_HIGH_JUNCTION.getValue()-100),false);
            robot.openClaw(false);
            for (int i = 0; i < 4; i++) {
                robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue(),false);
                robot.cycleLiftArmToCycle(true);
                robot.delay(1);
                robot.wideClaw();
                robot.delay(0.5);
                robot.liftToPosition((int) stackPos[i + 1]);
                robot.followTrajectorySequenceAsync(pickupTrajectory2);
                robot.waitForFinish();
                robot.closeClaw(false);
                robot.waitForFinish();
                robot.raiseLiftArmToOuttake(true);
                robot.delay(0.5);
                robot.liftToPosition(LIFT_HIGH_JUNCTION);
                robot.followTrajectorySequenceAsync(dropTrajectory);
                robot.waitForFinish();
                robot.liftToPosition((int) (LIFT_HIGH_JUNCTION.getValue()-100),false);
                robot.openClaw();
                robot.waitForFinish();
            }
            robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue(),false);
            robot.lowerLiftArmToIntake(true);
            robot.delay(1);
            robot.wideClaw();
            robot.delay(0.5);
            robot.liftToPosition(0);

            if (dummyP == 1) {
                robot.followTrajectorySequenceAsync(park1trajectory);
            } else if (dummyP == 3) {
                robot.followTrajectoryAsync(park3trajectory);
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
