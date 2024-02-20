package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "CenterStageAutonomous_RED", group = "Final Autonomous")
public class CenterStageAutnomous_RED extends LinearOpMode {

    protected SampleMecanumDrive drive;
    protected RoadRunnerCommand_RED RR_Red;
    protected RoadRunnerSubsystem_RED.Randomization rand;

    public Pose2d HomePose_SHORT = new Pose2d(RoadRunnerSubsystem_RED.Tile/2, 3 * RoadRunnerSubsystem_RED.TileInverted + 6.93 + 2.56, Math.toRadians(90));
    public Pose2d HomePose_LONG = new Pose2d(1.5 * RoadRunnerSubsystem_RED.TileInverted, 3 * RoadRunnerSubsystem_RED.TileInverted + 6.93 + 2.56, Math.toRadians(90));

    @Override
    public void runOpMode(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        RR_Red = new RoadRunnerCommand_RED(drive, hardwareMap, HomePose_LONG, RoadRunnerSubsystem_RED.StartingPosition.LONG,
                RoadRunnerSubsystem_RED.Path.INNER, RoadRunnerSubsystem_RED.PixelStack.INNER, RoadRunnerSubsystem_RED.ParkingPosition.OUTER, telemetry);

        rand = RoadRunnerSubsystem_RED.Randomization.LEFT;

        RR_Red.spikeRandomizationPath(rand);
        RR_Red.cycle();
        RR_Red.parking();
        RR_Red.TrajectoryInit();

        waitForStart();

        drive.followTrajectorySequence(RR_Red.getSpike(rand).build());
        drive.followTrajectorySequence(RR_Red.getCycle().build());
        drive.followTrajectorySequence(RR_Red.getParking().build());

    }
}
