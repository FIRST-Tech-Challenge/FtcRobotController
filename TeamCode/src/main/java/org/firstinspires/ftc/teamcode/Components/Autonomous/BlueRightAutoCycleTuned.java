package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

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

    public static double dummyxi = -12.5, dummyyi = 55;
    public static double dummyxi2 = -12.5, dummyyi2 = 13;


    public static double dummyx = -23.5, dummyy = 6, dummya = 270;
    public static double dummyx2 = -23.5, dummyy2 =11, dummya2 = 270;
    public static double dummyxd = -23.5, dummyyd = 6, dummyad = 270;
    public static double dummyx2i = -23.5, dummyy2i =11, dummya2i = 270;
    public static double dummyx3i = -23.5, dummyy3i =9, dummya3i = 270;
    public static double dummyx3 = -38, dummyy3 =10.1, dummya3 = 180;
    public static double dummyx4 = -63.5, dummyy4 =10.1, dummya4 = 180;

    public static double dummyX = -12, dummyY = 11, dummyA = 180;

    public static double dummyX2 = -35, dummyY2 = 11, dummyA2 = 180;

    public static double dummyX3 = -55, dummyY3 = 11, dummyA3 = 180;
    double[] stackPos = {470, 370, 240, 120,0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, Math.toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, Math.toRadians(90)))
                .setReversed(true).splineToSplineHeading(new Pose2d(-35, 40, Math.toRadians(85)), Math.toRadians(265))
                .splineToSplineHeading(new Pose2d(-35, 20, Math.toRadians(100)), Math.toRadians(280))
                .splineToSplineHeading(new Pose2d(-28,7,Math.toRadians(120)),Math.toRadians(300))
                .build();
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-28,7,Math.toRadians(120)))
                .splineToSplineHeading(new Pose2d(-50.5, 11.75, Math.toRadians(180)), Math.toRadians(180))
        .splineToSplineHeading(new Pose2d(-62.5, 11.75, Math.toRadians(180)), Math.toRadians(180))
                .build();
        TrajectorySequence dropTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-62.5, 11.75, Math.toRadians(180))).setReversed(true)
                .splineToSplineHeading(new Pose2d(-32,6.5,Math.toRadians(140)),Math.toRadians(320))
                .build();
        TrajectorySequence pickupTrajectory2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-30,5.5,Math.toRadians(140)))
                .splineToSplineHeading(new Pose2d(-45.5, 11.75, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-63.5, 11.75, Math.toRadians(180)), Math.toRadians(180))
                .build();
        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-30,5.5,Math.toRadians(140)))
                .splineToSplineHeading(new Pose2d(-36, 33,Math.toRadians(90)),Math.toRadians(90))
                .build();
        Trajectory park1trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(-36, 33,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-12, 33))
                .build();

        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2, dummyy2, Math.toRadians(dummya)))
                .lineToLinearHeading(new Pose2d(dummyX2, dummyY2, Math.toRadians(dummyA2)))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(-36, 33,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-57, 33, Math.toRadians(90)))
                .build();
        while (!isStarted()) {
            telemetry.addData("pos", robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();

        }
        resetRuntime();
        dummyP = robot.cv.getPosition();

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested() && getRuntime() < 28) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(initialtrajectory);
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.raiseLiftArmToOuttake(true);
            robot.waitForFinish();
            robot.openClaw(false);
            robot.followTrajectorySequenceAsync(pickupTrajectory);
            robot.delay(1);
            robot.liftToPosition((int) stackPos[0]);
            robot.delay(1.8);
            robot.lowerLiftArmToIntake(true);
            robot.closeClaw(false);
            robot.waitForFinish();
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.delay(0.4);
            robot.followTrajectorySequenceAsync(dropTrajectory);
            robot.raiseLiftArmToOuttake();
            robot.waitForFinish();
            robot.openClaw(false);
            for(int i=0;i<4;i++) {
                robot.followTrajectorySequenceAsync(pickupTrajectory2);
                robot.delay(1);
                robot.liftToPosition((int) stackPos[i+1]);
                robot.lowerLiftArmToIntake(true);
                robot.closeClaw(false);
                robot.followTrajectorySequenceAsync(dropTrajectory);
                robot.liftToPosition(LIFT_HIGH_JUNCTION);
                robot.raiseLiftArmToOuttake();
                robot.waitForFinish();
                robot.openClaw(true);
            }
            robot.followTrajectorySequenceAsync(parkTrajectory);
            robot.delay(1);
            robot.liftToPosition(0);
            robot.lowerLiftArmToIntake(true);

            if (dummyP == 1) {
                robot.followTrajectoryAsync(park1trajectory);
            } else if (dummyP == 3) {
                robot.followTrajectoryAsync(park3trajectory);
            } else {
//                robot.followTrajectoryAsync(park2trajectory);

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
