package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Lift.LiftConstants.LIFT_GROUND;
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
@Autonomous(name = "BlueRightAutoFar")


public class BlueRightAutoFar extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dummyx = -23.5, dummyy = 5, dummya = 270;
    public static double dummyx2 = -23.5, dummyy2 =8, dummya2 = 270;

    public static double dummyX = -10, dummyY = 10, dummyA = 180;

    public static double dummyX2 = -33, dummyY2 = 10, dummyA2 = 180;

    public static double dummyX3 = -55, dummyY3 = 10, dummyA3 = 180;

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, Math.toRadians(270));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        while(getRuntime()<5){
        }
        waitForStart();
        dummyP = robot.cv.getPosition();
        if (isStopRequested()) return;
        Trajectory initialtrajectory = robot.roadrun.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-13, 55))
                .build();
        Trajectory initialtrajectory2 = robot.roadrun.trajectoryBuilder(new Pose2d(-13,55, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-13, 10))
                .build();
        Trajectory preloadtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(-13,10, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(dummyx,dummyy))
                .build();
        Trajectory backtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx,dummyy, Math.toRadians(dummya)))
                .lineToConstantHeading(new Vector2d(dummyx2, dummyy2))
                .build();
        Trajectory park1trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(dummyA2)))
                .lineToConstantHeading(new Vector2d(dummyX, dummyY))
                .build();

        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(dummya)))
                .lineToLinearHeading(new Pose2d(dummyX2, dummyY2,Math.toRadians(dummyA2)))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(dummya2)))
                .lineToConstantHeading(new Vector2d(dummyX3, dummyY3))
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
            robot.liftToPosition(LIFT_GROUND);
            robot.delay(0.8);
            robot.lowerLiftArmToIntake(true);
            robot.followTrajectoryAsync(backtrajectory);

            if (dummyP == 1) {
                robot.followTrajectoryAsync(park1trajectory);
            }
            else if (dummyP == 3) {
                robot.followTrajectoryAsync(park3trajectory);
            }
            else{
                robot.followTrajectoryAsync(park2trajectory);

            }

            robot.setFirstLoop(false);
            robot.liftToTarget();
            robot.roadrun.update();
            robot.updateClawStates();
        }
        robot.stop();
        if(getRuntime()>29.8){
            stop();
        }
    }
}
