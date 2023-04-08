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
@Autonomous(name = "TeleOdoErrorTest")


public class TeleOdoErrorTest extends LinearOpMode {

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(41, 63.25, Math.toRadians(90));
        robot.setPoseEstimate(startPose);

        TrajectorySequence pretrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(41,63.25, Math.toRadians(90)))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(35, 40.25, toRadians(90)))
                .build();

        TrajectorySequence testtrajectory = robot.roadrun.trajectorySequenceBuilder(new Pose2d(41,63.25, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(35, 0, toRadians(90)))
                .lineToSplineHeading(new Pose2d(35, -12, toRadians(0)))

                .lineToLinearHeading(new Pose2d(-23.5, -12, toRadians(0)))
                .lineToSplineHeading(new Pose2d(-35, -12, toRadians(270)))

                .lineToLinearHeading(new Pose2d(-35, 47, toRadians(270)))
                .lineToSplineHeading(new Pose2d(-35, 59, toRadians(180)))

                .lineToLinearHeading(new Pose2d(23.5, 59, toRadians(180)))
                .lineToSplineHeading(new Pose2d(35, 59, toRadians(90)))
                .build();
        resetRuntime();

        while(!isStarted()){
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
//            telemetry.addData("ANGLE:", robot.getAngleToConeStack());
            telemetry.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();
            robot.heartbeatRed();
        }
        resetRuntime();
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            logger.loopcounter++;
            if (logger.loopcounter == 1) {
                robot.followTrajectorySequenceAsync(pretrajectory);
            }
            robot.followTrajectorySequenceAsync(testtrajectory);

            robot.setFirstLoop(false);
            robot.liftToTargetAuto();
            robot.roadrun.update();
            robot.updateClawStates();
            robot.updateLiftArmStates();

        }
        robot.stop();
    }
}
