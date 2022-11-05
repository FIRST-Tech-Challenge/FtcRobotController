package org.firstinspires.ftc.teamcode.Components.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;

@Config
@Autonomous(name = "BlueLeftAuto")


public class BlueLeftAuto extends LinearOpMode {
    public static Pose2d startPose = new Pose2d(36, 72, Math.toRadians(180));

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);

        //detectSignal();
//        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.roadrun.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;
        Trajectory initialtrajectory = robot.roadrun.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(15, 60))
                .build();
        Trajectory preloadtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(12,72, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-6, 35,Math.toRadians(-45)))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(6,35, Math.toRadians(-45)))
                .lineToSplineHeading(new Pose2d(12, 0,Math.toRadians(0)))
                .build();

        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(6,35, Math.toRadians(-45)))
                .lineToSplineHeading(new Pose2d(36, 0,Math.toRadians(0)))
                .build();

        Trajectory park1trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(6,35, Math.toRadians(-45)))
                .lineToSplineHeading(new Pose2d(60, 0,Math.toRadians(0)))
                .build();

        while (opModeIsActive()) {
            robot.followTrajectoryAsync(initialtrajectory);
            robot.followTrajectoryAsync(preloadtrajectory);
            //if signal variable is 1:
            robot.followTrajectoryAsync(park1trajectory);

            //if signal variable is 2:
            robot.followTrajectoryAsync(park2trajectory);

            //if signal variable is 3:
            robot.followTrajectoryAsync(park3trajectory);

            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}
