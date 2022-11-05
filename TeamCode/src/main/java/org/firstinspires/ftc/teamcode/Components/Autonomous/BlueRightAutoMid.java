package org.firstinspires.ftc.teamcode.Components.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "BlueRightAutoMid")


public class BlueRightAutoMid extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dummyxi = -35, dummyyi =50, dummyai = 270;

    public static double dummyx = -27, dummyy =32, dummya = 305;

    public static double dummyX = -12, dummyY =35, dummyA = 90;

    public static double dummyX2 = -33, dummyY2 =35, dummyA2 = 90;

    public static double dummyX3 = -55, dummyY3 =35, dummyA3 = 90;

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        //        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.2, 62.25, Math.toRadians(270));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable

        waitForStart();

        if (isStopRequested()) return;
        Trajectory initialtrajectory = robot.roadrun.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(dummyxi, dummyyi))
                .build();
        Trajectory preloadtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyxi,dummyyi, Math.toRadians(dummyai)))
                .lineToLinearHeading(new Pose2d(dummyx, dummyy, Math.toRadians(dummya)))
                .build();

        Trajectory park1trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx,dummyy, Math.toRadians(dummya)))
                .lineToLinearHeading(new Pose2d(dummyX, dummyY,Math.toRadians(dummyA)))
                .build();

        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyX,dummyY, Math.toRadians(dummyA)))
                .lineToLinearHeading(new Pose2d(dummyX2, dummyY2,Math.toRadians(dummyA2)))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyX2,dummyY2, Math.toRadians(dummyA2)))
                .lineToLinearHeading(new Pose2d(dummyX3, dummyY3,Math.toRadians(dummyA3)))
                .build();

        while (opModeIsActive()) {
            robot.followTrajectoryAsync(initialtrajectory);
            robot.followTrajectoryAsync(preloadtrajectory);
//            robot.liftToPosition(LIFT_HIGH_JUNCTION);
//            robot.openClaw(false);
            robot.followTrajectoryAsync(park2trajectory);
            if (dummyP == 1) {
                robot.followTrajectoryAsync(park1trajectory);
            }
            else if (dummyP == 3) {
                robot.followTrajectoryAsync(park3trajectory);
            }
            sleep(1000);

            robot.setFirstLoop(false);
            robot.roadrun.update();
        }
    }
}
