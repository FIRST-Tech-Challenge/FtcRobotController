package org.firstinspires.ftc.teamcode.Old.PowerPlay.Autonomous;

import static org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Lift.LiftConstants.LIFT_GROUND;
import static org.firstinspires.ftc.teamcode.Old.PowerPlay.Components.Lift.LiftConstants.LIFT_HIGH_JUNCTION;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Old.PowerPlay.Robot.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
@Disabled
@Config
@Autonomous(name = "RedRightAutoHighTuned")


public class RedRightAutoHighTuned extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public static double dummyx = 0.0, dummyy =28, dummya = 270;
    public static double dummyx2 = 0.0, dummyy2 =34, dummya2 = 280;

    public static double dummyX = -12, dummyY =36, dummyA = 90;

    public static double dummyX2 = -33, dummyY2 =36, dummyA2 = 90;

    public static double dummyX3 = -56, dummyY3 =34, dummyA3 = 90;

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(40.35, 62.25, Math.toRadians(270));
        robot.roadrun.setPoseEstimate(startPose);

        //detectSignal();
        //store in variable
        robot.cv.observeSleeve();
        while(!isStarted()){
            telemetry.addData("pos",robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
        }
        waitForStart();
        dummyP = robot.cv.getPosition();

        if (isStopRequested()) return;
        Trajectory initialtrajectory = robot.roadrun.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-13, 55))
                .build();
        Trajectory initialtrajectory2 = robot.roadrun.trajectoryBuilder(new Pose2d(-13,55, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(-13, 37))
                .build();
        Trajectory preloadtrajectory1 = robot.roadrun.trajectoryBuilder(new Pose2d(-13,37, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(0,33, Math.toRadians(dummya)))
                .build();
        Trajectory preloadtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(0,33, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(dummyx,dummyy, Math.toRadians(dummya)))
                .build();
        Trajectory backtrajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx,dummyy, Math.toRadians(dummya)))
                .lineToLinearHeading(new Pose2d(dummyx2, dummyy2,Math.toRadians(dummya2)))
                .build();
        Trajectory park1trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(dummya2)))
                .lineToLinearHeading(new Pose2d(dummyX, dummyY,Math.toRadians(dummyA)))
                .build();

        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(dummya2)))
                .lineToLinearHeading(new Pose2d(dummyX2, dummyY2,Math.toRadians(dummyA2)))
                .build();

        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(dummyx2,dummyy2, Math.toRadians(270)))
                .lineToConstantHeading(new Vector2d(dummyX3, dummyY3))
                .build();

        while (opModeIsActive() && !isStopRequested() && getRuntime()<28) {
            logger.loopCounter++;
            robot.followTrajectoryAsync(initialtrajectory);
            robot.liftToPosition(LIFT_HIGH_JUNCTION);
            robot.raiseLiftArmToOuttake(true);
            robot.followTrajectoryAsync(initialtrajectory2);
            robot.followTrajectoryAsync(preloadtrajectory1);
            robot.followTrajectoryAsync(preloadtrajectory);
            robot.liftToPosition(1300);
            robot.waitForFinish();
            robot.openClaw(false);
            robot.liftToPosition(LIFT_GROUND);
            robot.delay(0.8);
            robot.lowerLiftArmToIntake(true);
            robot.followTrajectoryAsync(backtrajectory);

            if (dummyP == 1) {
                robot.followTrajectoryAsync(park1trajectory);
            }
            else if (dummyP == 2) {
                robot.followTrajectoryAsync(park2trajectory);
            }
            else if (dummyP == 3) {
                robot.followTrajectoryAsync(park3trajectory);
            }

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
