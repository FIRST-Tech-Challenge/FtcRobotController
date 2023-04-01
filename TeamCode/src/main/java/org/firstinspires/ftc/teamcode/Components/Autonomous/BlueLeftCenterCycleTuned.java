package org.firstinspires.ftc.teamcode.Components.Autonomous;

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
@Autonomous(name = "BlueLeftCenterCycleTuned")


public class BlueLeftCenterCycleTuned extends LinearOpMode {
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
    public static double dropX=10.4, dropY=18.3;
    double[] stackPos = {420*1.03,330*1.03,235*1.03,80*1.03,0};

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(41, 63.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);


        robot.cv.observeSleeve();

//        Trajectory initialtrajectory2 = robot.roadrun.trajectoryBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(36, 57))
//                .build();
        TrajectorySequence preloadtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(42,63.25, Math.toRadians(90)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(37, 40.25, toRadians(90)), toRadians(270))
                .splineTo(new Vector2d(36, 22), toRadians(275))
                .splineToSplineHeading(new Pose2d(26, 4.5, toRadians(50)), toRadians(230))
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
        TrajectorySequence pickupTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(26.5,7.2,Math.toRadians(50)))
                .setReversed(false)
                .splineTo(new Vector2d(62.7, 11.25), Math.toRadians(0))
                .addTemporalMarker(robot::done)
                .build();

        TrajectorySequence pickupTrajectory2 = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY,Math.toRadians(40)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(48, 11.75+(robot.getVoltage()-12)/2, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63, 11.75+(robot.getVoltage()-12)/2), Math.toRadians(0))
                .addTemporalMarker(robot::done)
                .build();
//        TrajectorySequence approachTrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(dummyx3,dummyy3,Math.toRadians(dummya3))).
//                lineToConstantHeading(new Vector2d(dummyx4-5,dummyy4))
//                .lineToConstantHeading(new Vector2d(dummyx4,dummyy4),
//                        SampleMecanumDrive.getVelocityConstraint(5,30,30),
//                        SampleMecanumDrive.getAccelerationConstraint(30))
//                .build();
        ArrayList<TrajectorySequence> dropTrajectory = new ArrayList<>();
        ArrayList<TrajectorySequence> pick = new ArrayList<>();

        for(int i=0;i<5;i++){
            dropTrajectory.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(64,11.75,Math.toRadians(0)))
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(dropX, dropY, Math.toRadians(335)), Math.toRadians(155))
                    .addTemporalMarker(robot::done)
                    .build());
        }
        for(int i=0;i<5;i++){
            pick.add(robot.roadrun.trajectorySequenceBuilder(new Pose2d(dropX,dropY,Math.toRadians(335)))
                    .setReversed(false)
//                    .splineToSplineHeading(new Pose2d(48, 11.75, Math.toRadians(0)), Math.toRadians(0))
                    .splineTo(new Vector2d(62.7, 11.5+0.1*i), Math.toRadians(0))
                    .addTemporalMarker(robot::done)
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
                .splineToSplineHeading(new Pose2d(61, 12, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence park2trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(40)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(36, 17, Math.toRadians(90)), Math.toRadians(90))
                .build();

        TrajectorySequence park3trajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(30,5, Math.toRadians(40)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(35, 7), Math.toRadians(130))
                .splineTo(new Vector2d(12, 16), Math.toRadians(180))
                .build();
        resetRuntime();
        robot.cp1shot();
        while(!isStarted()){
            telemetry.addData("pos",robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
//            telemetry.addData("ANGLE:", robot.getAngleToConeStack());
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
        while (opModeIsActive() && !isStopRequested() && getRuntime()<29.8) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(preloadtrajectory);
//            robot.delay(1.2);
//            robot.updateTrajectoryWithCam();
//            robot.delay(0.3);
//            robot.raiseLiftArmToOuttake(true);
//            robot.delay(0.5);
//            robot.liftToPosition(LIFT_HIGH_JUNCTION);
//            robot.openClaw(  false);
//            robot.delay(0.4);
//            robot.cycleLiftArmToCycle(true);
//            robot.delay(0.5);
//            robot.wideClaw();
//            robot.delay(0.5);
//            robot.iterateConestackUp();
            robot.followTrajectorySequenceAsync(pickupTrajectory);
//            robot.closeClaw(false);
            robot.followTrajectorySequenceAsync(dropTrajectory.get(0));
//            robot.liftToPosition(LIFT_HIGH_JUNCTION);
//            robot.delay(0.55);
//            robot.raiseLiftArmToOuttake(true);
////            robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue()-250, false);
//            robot.delay(0.2);
//            robot.wideClaw(false);
//            robot.delay(0.25);
            robot.followTrajectorySequenceAsync(pick.get(0));
//            robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue() + 50, true);
            for (int i = 0; i < 3; i++) {
//                robot.delay(0.15);
//                robot.cycleLiftArmToCycle(true);
//                robot.delay(0.15);
//                robot.wideClaw();
//                robot.delay(0.35);
//                if(i!=3){
//                    robot.iterateConestackDown();
//                }
//                else{
//                    robot.liftToPosition(0);
//                }
//                robot.closeClaw(false);
                robot.followTrajectorySequenceAsync(dropTrajectory.get(i));
//                robot.delay(0.3);
//                robot.updateTrajectoryWithCam();
//                robot.delay(0.03+0.005*(3-i));
//                robot.liftToPosition(LIFT_HIGH_JUNCTION);
//                robot.delay(0.36+0.005*(3-i));
//                robot.raiseLiftArmToOuttake(true);
//                robot.delay(0.2);
////                robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue()-250, false);
////                robot.delay(0.65);
//                robot.wideClaw(false);
//                robot.delay(0.15);
                robot.followTrajectorySequenceAsync(pick.get(i));
//                robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue() + 50, true);
            }

//            robot.delay(0.15);
//            robot.lowerLiftArmToIntake(true);
//            robot.delay(0.5);
//            robot.wideClaw();
//            robot.delay(0.25);
//            robot.liftToPosition(0);
//            robot.closeClaw(false);
            robot.followTrajectorySequenceAsync(dropTrajectory.get(3));
//            robot.delay(0.3);
            robot.updateTrajectoryWithCam();
//            robot.delay(0.03+0.005*(3-3));
//            robot.liftToPosition(LIFT_HIGH_JUNCTION);
//            robot.delay(0.36+0.005*(3-3));
//            robot.raiseLiftArmToOuttake(true);
////            robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue()-220, false);
//            robot.delay(0.35);
//            robot.wideClaw(false);
//            robot.delay(0.1);

//            if (dummyP == 1) {
//                robot.followTrajectorySequenceAsync(park1trajectory);
//            } else if (dummyP == 3) {
//                robot.followTrajectorySequenceAsync(park3trajectory);
//            } else {
//                robot.followTrajectorySequenceAsync(park2trajectory);
//            }
//
//            robot.delay(0.3);

//            robot.liftToPosition((int)LIFT_HIGH_JUNCTION.getValue() + 50, true);


//            robot.lowerLiftArmToIntake(true);
//            robot.delay(1.6);
//            robot.wideClaw();
//            robot.delay(0.3);
//            robot.liftToPosition(0);


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
