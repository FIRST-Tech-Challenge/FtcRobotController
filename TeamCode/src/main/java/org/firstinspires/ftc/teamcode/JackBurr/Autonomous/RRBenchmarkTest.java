package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.JackBurr.Odometry.Roadrunner.PinpointDrive;

@Autonomous
public class RRBenchmarkTest extends LinearOpMode {
    public static Pose2d startPose = new Pose2d(36, 62, Math.toRadians(-90)); // (60,0), 180
    public static Vector2d position1 = new Vector2d(36, -48); //(x+0, y-110) (forward 110 inches)
    public static Vector2d position2 = new Vector2d(-50, -48);
    public static Vector2d position3 = new Vector2d(-50, 52);
    public static Vector2d position4 = new Vector2d(36, 52);
    public PinpointDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new PinpointDrive(hardwareMap, startPose);
        TrajectoryActionBuilder traj1Builder = drive.actionBuilder(startPose)
                .strafeTo(position1)
                .waitSeconds(2)
                .strafeTo(position2)
                .waitSeconds(2)
                .strafeTo(position3)
                .waitSeconds(2)
                .strafeTo(position4);
        Action action1 = traj1Builder.build();
        waitForStart();
        Actions.runBlocking(action1);
    }
}
