package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PinpointDrive;

@Autonomous
public class RRBenchmarkTest extends LinearOpMode {
    public static Pose2d startPose = new Pose2d(36, 62, Math.toRadians(-90)); // (60,0), 180
    public static Vector2d position1 = new Vector2d(36, -38);
    public PinpointDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new PinpointDrive(hardwareMap, startPose);
        TrajectoryActionBuilder traj1Builder = drive.actionBuilder(startPose)
                .splineTo(position1, startPose.heading);
        Action action1 = traj1Builder.build();
        waitForStart();
        Actions.runBlocking(action1);
    }
}
