package org.firstinspires.ftc.teamcode.Components.Autonomous;

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

@Config
@Autonomous(name = "BlueRightAutoCycle")


public class BlueRightAutoCycle extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dummyx = -23, dummyy = 3.5, dummya = 270;
    public static double dummyx2 = -23, dummyy2 =10, dummya2 = 270;
    public static double dummyx3 = -50, dummyy3 =10, dummya3 = 180;
    public static double dummyx4 = -62.5, dummyy4 =10, dummya4 = 180;

    public static double dummyX = -10, dummyY = 10, dummyA = 180;

    public static double dummyX2 = -33, dummyY2 = 10, dummyA2 = 180;

    public static double dummyX3 = -55, dummyY3 = 10, dummyA3 = 180;
    double[] stackPos = {500,400,173,53,0};

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, Math.toRadians(270));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        waitForStart();
        resetRuntime();
        dummyP = robot.cv.getPosition();

        if (isStopRequested()) return;
        Trajectory initialtrajectory = robot.roadrun.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-13, 55))
                .build();
        Trajectory initialtrajectory2 = robot.roadrun.trajectoryBuilder(new Pose2d(-13,55, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-15, 13))
                .build();
        Trajectory preloadtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(-15,13, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(dummyx,dummyy,Math.toRadians(dummya)))
                .build();
        Trajectory backtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx,dummyy, Math.toRadians(dummya)))
                .lineToConstantHeading(new Vector2d(dummyx2, dummyy2))
                .build();
        Trajectory pickupTrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2,Math.toRadians(dummya2))).
                lineToLinearHeading(new Pose2d(dummyx3,dummyy3,Math.toRadians(dummya3)))
                        .build();
        Trajectory approachTrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx3,dummyy3,Math.toRadians(dummya3))).
                lineToConstantHeading(new Vector2d(dummyx4,dummyy4))
                .build();
        Trajectory dropTrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx4,dummyy4,Math.toRadians(dummya4))).
                lineToLinearHeading(new Pose2d(dummyx2,dummyy2,Math.toRadians(dummya2)))
                .build();
        Trajectory dropTrajectory2 = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2,Math.toRadians(dummya2))).
                lineToLinearHeading(new Pose2d(dummyx,dummyy,Math.toRadians(dummya)))
                .build();
        Trajectory park1trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx,dummyy, Math.toRadians(dummya)))
                .lineToConstantHeading(new Vector2d(dummyX, dummyY),
                        SampleMecanumDrive.getVelocityConstraint(5,30,30),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();

        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(dummya)))
                .lineToLinearHeading(new Pose2d(dummyX2, dummyY2,Math.toRadians(dummyA2)))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(dummya2)))
                .lineToLinearHeading(new Pose2d(dummyX3, dummyY3,dummyA3))
                .build();

        while (opModeIsActive() && !isStopRequested() && getRuntime()<28) {
            logger.loopcounter++;
            robot.followTrajectoryAsync(initialtrajectory);
            robot.delay(1);
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.raiseLiftArmToOuttake(true);
            robot.followTrajectoryAsync(initialtrajectory2);
            robot.followTrajectoryAsync(preloadtrajectory);
            robot.waitForFinish();
            robot.openClaw(false);
//            robot.liftToPosition((int)stackPos[0]);
//            robot.delay(0.8);
//            robot.lowerLiftArmToIntake(true);
//            robot.followTrajectoryAsync(backtrajectory);
//            for(int i=0;i<2;i++){
//                robot.followTrajectoryAsync(pickupTrajectory);
//                robot.followTrajectoryAsync(approachTrajectory,true);
//                robot.closeClaw(true);
//                robot.waitForFinish();
//                robot.liftToPosition((int)(stackPos[i]+400),false);
//                robot.followTrajectoryAsync(dropTrajectory);
//                robot.liftToPosition(LIFT_HIGH_JUNCTION);
//                robot.raiseLiftArmToOuttake();
//                robot.followTrajectoryAsync(dropTrajectory2);
//                robot.waitForFinish();
//                robot.openClaw(true);
//                robot.liftToPosition((int)stackPos[i+1]);
//                robot.delay(0.8);
//                robot.lowerLiftArmToIntake(true);
//                robot.followTrajectoryAsync(backtrajectory);
//            }
//
//            if (dummyP == 1) {
//                robot.followTrajectoryAsync(park1trajectory);
//            }
//            else if (dummyP == 3) {
//                robot.followTrajectoryAsync(park3trajectory);
//            }
//            else{
//                robot.followTrajectoryAsync(park2trajectory);
//
//            }

            robot.setFirstLoop(false);
            robot.liftToTargetAuto();
            robot.roadrun.update();
            robot.updateClawStates();
        }
        robot.stop();
        if(getRuntime()>29.8){
            stop();
        }
    }
}
