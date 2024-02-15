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

    public final double Tile = 24; /*-inches-*/
    public final double TileInverted = -24; /*-inches-*/
    public final double RobotX = 12.5984252; /*-inches-*/
    public final double RobotY = 16.9291339; /*-inches-*/
    public final double BackdropDistance = 0; /*-inches-*/

    public Pose2d HomePose_SHORT = new Pose2d(Tile/2, 3 * TileInverted + (RobotY/2) + 3, Math.toRadians(90));
    public Pose2d HomePose_LONG = new Pose2d(1.5 * TileInverted, 3 * TileInverted + (RobotY/2), Math.toRadians(90));

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        RR_Red = new RoadRunnerCommand_RED(hardwareMap, HomePose_SHORT, RoadRunnerSubsystem_RED.StartingPosition.SHORT,
                RoadRunnerSubsystem_RED.Path.INNER, RoadRunnerSubsystem_RED.PixelStack.INNER, RoadRunnerSubsystem_RED.ParkingPosition.OUTER);

        rand = RoadRunnerSubsystem_RED.Randomization.CENTER;

        waitForStart();

        drive.setPoseEstimate(HomePose_SHORT);

        drive.update();
        drive.followTrajectorySequence(RR_Red.spikeRandomizationPath(rand).build());
        drive.update();
        drive.followTrajectorySequence(RR_Red.cycle(rand).build());
        drive.update();
        drive.followTrajectorySequence(RR_Red.parking().build());
    }
}
