package org.firstinspires.ftc.teamcode.Old.PowerPlay.Autonomous;

import static org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
//@Disabled
@Disabled

@Config
@Autonomous(name = "RedLeftAutoCycleTuned")


public class RedLeftAutoCycleTuned extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dummyxi = 12.5, dummyyi = 55;
    public static double dummyxi2 = 12.5, dummyyi2 = 13;


    public static double dummyx = 23.5, dummyy = 4.5, dummya = 270; //coord for pole 1: 23.5 , 5
    public static double dummyx2 = 23.5, dummyy2 =11.5, dummya2 = 270; //coord for pole 1: 23.5 , 5
    public static double dummyxd = 22.25, dummyyd = 4, dummyad = 270; //coord for pole 2: 22.25 , 4.5
    public static double dummyx2i = 22.25, dummyy2i =10, dummya2i = 270; //coord for pole 2: 22.25 , 4.5
    public static double dummyx3i = 22.25, dummyy3i =8, dummya3i = 270;//coord for pole 2: 22.25 , 4.5
    public static double dummyx3 = 38, dummyy3 =10.1, dummya3 = 0;
    public static double dummyx4 = 64, dummyy4 =10.1, dummya4 = 0;

    public static double dummyX = 12, dummyY = 11, dummyA = 0;

    public static double dummyX2 = 35, dummyY2 = 11, dummyA2 = 0;

    public static double dummyX3 = 53, dummyY3 = 11, dummyA3 = 0;
    public static double dropX=30.75, dropY=5.9;
    double[] stackPos = {400*1.03,330*1.03,235*1.03,80*1.03,0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(42, 63.25, Math.toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);


        robot.cv.observeSleeve();

//        Trajectory initialtrajectory2 = robot.roadrun.trajectoryBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(36, 57))
//                .build();
        TrajectorySequence preloadtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(42,63.5, Math.toRadians(90)))
                .setReversed(true).splineToSplineHeading(new Pose2d(38, 51, toRadians(70)), toRadians(250))
                .splineTo(new Vector2d(38, 18), toRadians(260))
                .splineToSplineHeading(new Pose2d(26.5, 7.2, Math.toRadians(55)), Math.toRadians(230))
                .build();
//        Trajectory preloadtrajectory2 = robot.roadrun.trajectoryBuilder(new Pose2d(37,50, Math.toRadians(70)))
//                .lineToConstantHeading(new Vector2d(36, 12))
//                .build();
//        Trajectory preloadtrajectory3 = robot.roadrun.trajectoryBuilder(new Pose2d(36,12, Math.toRadians(270)))
//                .lineToConstantHeading(new Vector2d(dummyx2, dummyy2))
//                .build();
//        Trajectory backtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx,dummyy, Math.toRadians(dummya)))
//                .lineToConstantHeading(new Vector2d(dummyx2, dummyy2))
//                .build();
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(26.9,7.6,Math.toRadians(55)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(48, 8.2 + (robot.getVoltage()-12)/1.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63.8, 13), Math.toRadians(0))
                .addTemporalMarker(robot::done)
                .build();

        TrajectorySequence pickupTrajectory2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY,Math.toRadians(40)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(48, 10+(robot.getVoltage()-12)/2, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63.8, 13), Math.toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
//        TrajectorySequence approachTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dummyx3,dummyy3,Math.toRadians(dummya3))).
//                lineToConstantHeading(new Vector2d(dummyx4-5,dummyy4))
//                .lineToConstantHeading(new Vector2d(dummyx4,dummyy4),
//                        SampleMecanumDrive.getVelocityConstraint(5,30,30),
//                        SampleMecanumDrive.getAccelerationConstraint(30))
//                .build();
        ArrayList<TrajectorySequence> dropTrajectory = new ArrayList<>();
        for(int i=0;i<5;i++){
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(63.8,13,Math.toRadians(0)))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(dropX-(i+1)*0.4, dropY+(i+1)*0.3, Math.toRadians(42)), Math.toRadians(222))
                    .UNSTABLE_addTemporalMarkerOffset(0.4,robot::done)
                    .build());
        }
//        TrajectorySequence testTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dummyx4,dummyy4,Math.toRadians(dummya4))).
//                spline(new Vector2d(dummyx2i,dummyy2i))
//                .build();
//        Trajectory dropTrajectory3 = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2i,dummyy2i,Math.toRadians(dummya4))).
//                lineToLinearHeading(new Pose2d(dummyx3i,dummyy3i,Math.toRadians(dummya2i)))
//                .build();
//        Trajectory dropTrajectory2 = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx3i,dummyy3i,Math.toRadians(dummya3i))).
//                lineToLinearHeading(new Pose2d(dummyxd,dummyyd,Math.toRadians(dummyad)))
//                .build();
        TrajectorySequence park1trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(40)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(59, 14, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(40)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(34, 14, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(40)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35, 7), Math.toRadians(130))
                .splineTo(new Vector2d(10, 16), Math.toRadians(180))
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

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested() && getRuntime()<29.8) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(preloadtrajectory);
            robot.delay(0.3);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.5);
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
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
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.delay(0.55);
            robot.raiseLiftArmToOuttake(true);
            robot.delay(0.25);
            robot.openClaw(false);
            for (int i = 0; i < 4; i++) {
                robot.followTrajectorySequenceAsync(pickupTrajectory2);
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
                robot.delay(0.0+0.005*(3-i));
                robot.liftToPosition(LIFT_HIGH_JUNCTION);
                robot.delay(0.3+0.005*(3-i));
                robot.raiseLiftArmToOuttake(true);
                robot.delay(0.15);
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
