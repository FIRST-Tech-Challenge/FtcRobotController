package org.firstinspires.ftc.teamcode.Components.Autonomous;

import static org.firstinspires.ftc.teamcode.Components.Claw.ClawStates.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static java.lang.Math.toRadians;

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
//@Disabled
@Config
@Autonomous(name = "BlueRightPark")


public class BlueRightPark extends LinearOpMode {
    private SampleMecanumDrive roadrun;

    public static double dummyP = 3;

    public void runOpMode() {
        PwPRobot robot = new PwPRobot(this, false);
        robot.roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pose2d startPose = new Pose2d(-29.6, 62.25, toRadians(90));
        robot.roadrun.setPoseEstimate(startPose);
        robot.cv.observeSleeve();
        TrajectorySequence parkTrajectory = robot.roadrun.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineTo(new Vector2d(-36, 62.25))
                .lineTo(new Vector2d(-36, 34))
                .build();
        Trajectory park1trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(-36,34,toRadians(0)))
                .lineTo(new Vector2d(-6, 34))
                .build();
//        Trajectory park2trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(-36,36,toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-36,36, toRadians(90)))
//                .build();
        Trajectory park3trajectory = robot.roadrun.trajectoryBuilder(new Pose2d(-36,34,toRadians(180)))
                .lineTo(new Vector2d(-57, 34))
                .build();
        while (!isStarted()) {
            telemetry.addData("pos", robot.cv.getPosition());
            telemetry.addData("CLAW_CLOSED:", CLAW_CLOSED.getStatus());
            telemetry.update();
            robot.updateLiftArmStates();
            //robot.leds.rainbowforest();

        }
        resetRuntime();
        dummyP = robot.cv.getPosition();

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested() && getRuntime() < 28) {
            logger.loopcounter++;
            robot.followTrajectorySequenceAsync(parkTrajectory);
            if (dummyP == 1) {
                //robot.leds.red();
                robot.followTrajectoryAsync(park1trajectory);
            } else if (dummyP == 3) {
                //robot.leds.green();
                robot.followTrajectoryAsync(park3trajectory);
            } else {
                //robot.leds.yellow();
//            robot.followTrajectoryAsync(park2trajectory);
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
