package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "CenterStageAutonomous_RED", group = "Final Autonomous")
public class CenterStageAutnomous_RED extends LinearOpMode {

    protected SampleMecanumDrive drive;
    protected RoadRunnerCommand_RED RR_Red;
    protected RoadRunnerSubsystem_RED.Randomization rand;

    public Pose2d HomePose_SHORT = new Pose2d(RoadRunnerSubsystem_RED.Tile/2, 3 * RoadRunnerSubsystem_RED.TileInverted + 6.93 + 2.56, Math.toRadians(90));
    public Pose2d HomePose_LONG = new Pose2d(1.5 * RoadRunnerSubsystem_RED.TileInverted, 3 * RoadRunnerSubsystem_RED.TileInverted + (RoadRunnerSubsystem_RED.RobotY/2), Math.toRadians(90));

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        RR_Red = new RoadRunnerCommand_RED(drive, hardwareMap, HomePose_SHORT, RoadRunnerSubsystem_RED.StartingPosition.SHORT,
                RoadRunnerSubsystem_RED.Path.INNER, RoadRunnerSubsystem_RED.PixelStack.INNER, RoadRunnerSubsystem_RED.ParkingPosition.OUTER);

        rand = RoadRunnerSubsystem_RED.Randomization.CENTER;

        waitForStart();

        drive.followTrajectorySequence(RR_Red.spikeRandomizationPath(rand).build());
        drive.followTrajectorySequence(RR_Red.cycle(rand).build());
        drive.followTrajectorySequence(RR_Red.parking().build());
    }
}
