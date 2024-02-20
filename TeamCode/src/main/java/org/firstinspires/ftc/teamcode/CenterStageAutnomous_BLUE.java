package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "CenterStageAutonomous_BLUE", group = "Final Autonomous")
public class CenterStageAutnomous_BLUE extends LinearOpMode {

    protected SampleMecanumDrive drive;
    protected RoadRunnerCommand_BLUE RR_Blue;
    protected RoadRunnerSubsystem_BLUE.Randomization rand;

    public Pose2d HomePose_SHORT = new Pose2d(RoadRunnerSubsystem_BLUE.Tile/2, 3 * RoadRunnerSubsystem_BLUE.Tile - 6.93 - 2.56, Math.toRadians(270));
    public Pose2d HomePose_LONG = new Pose2d(1.5 * RoadRunnerSubsystem_BLUE.TileInverted, 3 * RoadRunnerSubsystem_BLUE.TileInverted + (RoadRunnerSubsystem_BLUE.RobotY/2), Math.toRadians(90));

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        RR_Blue = new RoadRunnerCommand_BLUE(drive, hardwareMap, HomePose_SHORT, RoadRunnerSubsystem_BLUE.StartingPosition.SHORT,
                RoadRunnerSubsystem_BLUE.Path.INNER, RoadRunnerSubsystem_BLUE.PixelStack.INNER, RoadRunnerSubsystem_BLUE.ParkingPosition.OUTER, telemetry);

        rand = RoadRunnerSubsystem_BLUE.Randomization.LEFT;

        RR_Blue.spikeRandomizationPath(rand);
        RR_Blue.cycle();
        RR_Blue.parking();
        RR_Blue.TrajectoryInit();

        waitForStart();

        drive.followTrajectorySequence(RR_Blue.getSpike(rand).build());
        drive.followTrajectorySequence(RR_Blue.getCycle().build());
        drive.followTrajectorySequence(RR_Blue.getParking().build());
    }
}
