package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Config
@Autonomous(name = "BlueRightAutoCycleTunedCamera")


public class BlueRightAutoCycleTunedCamera extends LinearOpMode {

    public static double dummyP = 3;

    public static double dropX = -30.5, dropY = 3.5, dropA = toRadians(140), dropET = toRadians(310);

    public static double pickupX1 = -45.5, pickupY1 = 11.75, pickupA1 = toRadians(180), pickupET1 = toRadians(180);
    public static double pickupX2 = -62.9, pickupY2 = 11.75, pickupA2 = toRadians(180), pickupET2 = toRadians(180);
    double[] stackPos = {440, 330, 245, 100, 0};


    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        robot.cv.observeCone();
        TrajectorySequence initialtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.6, 62.25, toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-35.4, 34, toRadians(90)), toRadians(270))
                .splineToSplineHeading(new Pose2d(-30.4, 3.6, toRadians(140)), toRadians(310))
                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence pickupSecondPartTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-52, 11.75, toRadians(180)))
                .splineToSplineHeading(new Pose2d(-63, 11.75, toRadians(180)), toRadians(180))
//                .addTemporalMarker(robot::done)
                .build();
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-29.4, 3.2, toRadians(125)))
//                .splineToSplineHeading(new Pose2d(-45, 11.75, toRadians(180)), toRadians(178))
//                .splineToSplineHeading(new Pose2d(-45, 11,toRadians(180)), toRadians(180))
//                .splineTo(new Vector2d(-48,11), toRadians(180))
//                .splineTo(new Vector2d(-48,8), toRadians(180))
                .setTangentOffset(toRadians(toRadians(10) * robot.getVoltage() / 12))
//                .splineToSplineHeading(new Pose2d(new Vector2d(-55,10.5),toRadians(180)), toRadians(180))
                .splineToSplineHeading(new Pose2d(-63, 11.25, toRadians(180)), toRadians(180))
                .addTemporalMarker(() -> {
                    robot.done();
                })
                .build();
        ArrayList<TrajectorySequence> pick = new ArrayList<>();
        ArrayList<TrajectorySequence> dropTrajectory = new ArrayList<>();
        for (int i = 0; i < 5; i++) {
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(-64, 11.25, Math.toRadians(175)))
                    .setTangentOffset(toRadians(-180) + toRadians(-12) * robot.getVoltage() / 12)

//                    .splineToSplineHeading(new Pose2d(-38,9,toRadians(130)),toRadians(320))
                    .splineToSplineHeading(new Pose2d(dropX - (i + 1) * 0.4, dropY - (i + 1) * 1.1, Math.toRadians(130)), Math.toRadians(320))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        for (int i = 0; i < 5; i++) {
            pick.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, toRadians(125)))
//                .splineToSplineHeading(new Pose2d(-52.4, 5+(robot.getVoltage()-12)/1.5,toRadians(180)), toRadians(180))
//                .splineToSplineHeading(new Pose2d(-45, 11,toRadians(180)), toRadians(180))
//                .splineTo(new Vector2d(-50,11), toRadians(179))
                    .setTangentOffset(toRadians(10))
                .splineToSplineHeading(new Pose2d(new Vector2d(-55,11.75),toRadians(180)), toRadians(180))
                    .splineTo(new Vector2d(-64.5, 11.75), toRadians(180))
                    .addTemporalMarker(() -> {
                        robot.done();
                    })
                    .build());
        }
        TrajectorySequence pickupTrajectory2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, toRadians(120)))
//                .splineToSplineHeading(new Pose2d(-52.4, 5+(robot.getVoltage()-12)/1.5,toRadians(180)), toRadians(180))
//                .splineToSplineHeading(new Pose2d(-45, 11,toRadians(180)), toRadians(180))
//                .splineTo(new Vector2d(-50,11), toRadians(179))
                .setTangentOffset(toRadians(toRadians(10) * robot.getVoltage() / 12))
//                .splineToSplineHeading(new Pose2d(new Vector2d(-55,10.5),toRadians(180)), toRadians(180))
                .splineTo(new Vector2d(-63.5, 11.25), toRadians(180))
                .addTemporalMarker(() -> {
                    robot.done();
                })
                .build();
        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToSplineHeading(new Pose2d(-36, 33, toRadians(90)), toRadians(90))
                .build();
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .splineToLinearHeading(new Pose2d(dropX - 4, dropY + 5, toRadians(185)), toRadians(dropA))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 12, toRadians(180)), toRadians(0))
                .build();
        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(dropX - 7, dropY + 7, toRadians(185)), toRadians(dropA))
                .build();

        TrajectorySequence park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX, dropY, dropA))
//                .splineToSplineHeading(new Pose2d(-44, 11.75,toRadians(183)), toRadians(180))
                .splineTo(new Vector2d(-60, 12), toRadians(180))
                .build();
        resetRuntime();
        robot.cp1shot();
        while (!isStarted()) {
            telemetry.addData("pos", robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
            if (getRuntime() > 3) {
                int color = robot.cv.getPosition();
                dummyP = color;

                if (color == 1) {
                    robot.heartbeatRed();
                } else if (color == 2) {
                    robot.darkGreen();
                } else {
                    robot.violet();
                }
            }
        }
//        robot.cv.observeStick();

        waitForStart();
        resetRuntime();
//        dummyP = 1;

        if (isStopRequested()) return;
        robot.cv.observeStick();
        while (opModeIsActive() && !isStopRequested() && getRuntime() < 29.8) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(initialtrajectory);
//            robot.delay(2.2);
//            robot.updateTrajectoryWithCam();
            robot.delay(0.3);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.5);
            robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue());
//            robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue() - 180, false);
            robot.delay(0.15);
            robot.wideClaw(false);
            robot.splineTo(new Pose2d(-64.25, 11.5, toRadians(180)), toRadians(180), toRadians(-5));
//            robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue() + 50, true);
            robot.delay(0.4);
            robot.cycleLiftArmToCycle(true);
            robot.delay(0.8);
            robot.wideClaw();
            robot.delay(0.6);
            robot.liftToPosition((int) stackPos[0], true);
            robot.closeClaw(false);
            robot.splineTo(new Pose2d(dropX, dropY, toRadians(127)), toRadians(300), toRadians(-180));
//            robot.delay(0.4);
            robot.updateTrajectoryWithCam();
            robot.delay(0.01);
            robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue(), true);
            robot.delay(0.2);
            robot.raiseLiftArmToOuttake(true);
//            robot.delay(0.1);
//            robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue() - 180, false);
            robot.delay(0.25);
            robot.wideClaw(false);
            for (int i = 0; i < 4; i++) {
                robot.delay(0.2);
                robot.splineTo(new Pose2d(-64.25,11.5,toRadians(180)),toRadians(180),toRadians(-5));
//                robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue() + 50, true);

                robot.delay(0.4);
                if (i != 3) {
                    robot.cycleLiftArmToCycle(true);
                } else {
                    robot.cycleLiftArmToCycle(true);
                }
                robot.delay(0.5);
                robot.wideClaw();
                robot.delay(0.8);
                robot.liftToPosition((int) stackPos[i + 1]);
//                robot.updateTrajectoryWithCone();
                robot.closeClaw(false);
                if (false) {//claw no detect

                }
                robot.splineTo(new Pose2d(dropX, dropY, toRadians(127)), toRadians(300), toRadians(-180));
                robot.delay(0.9);
                robot.updateTrajectoryWithCam();
                robot.delay(0.1 + (3 - i) * 0.005);
                robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue());
                robot.delay(0.36 + 0.005 * (3 - i));
                robot.raiseLiftArmToOuttake(true);
//                robot.delay(0.1);
//                robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue() - 180, false);
                robot.delay(0.25);
                robot.wideClaw(false);
            }
            if (dummyP == 1) {
                robot.delay(0.4);
                robot.followTrajectorySequenceAsync(park1trajectory);
            } else if (dummyP == 3) {
                robot.delay(0.4);
                robot.followTrajectorySequenceAsync(park3trajectory);
            } else {
                robot.delay(0.4);
                robot.followTrajectorySequenceAsync(park2trajectory);
            }
            robot.liftToPosition((int) LIFT_HIGH_JUNCTION.getValue() + 50, true);
            robot.delay(0.3);
            robot.lowerLiftArmToIntake(true);
            robot.delay(1.5);
            robot.wideClaw();
            robot.delay(0.6);
            robot.liftToPosition(0);


            robot.queuedStop();
            robot.setFirstLoop(false);
            robot.liftToTargetAuto();
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
