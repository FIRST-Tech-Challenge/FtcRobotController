package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelFingerSubsystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "CenterStageAutonomous_BLUE", group = "Final Autonomous")
public class CenterStageAutnomous_BLUE extends CommandOpMode {

    private SampleMecanumDrive drive;
    private RoadRunnerCommand_BLUE RR_Blue;
    private RoadRunnerSubsystem_BLUE.Randomization rand;
    private AutonomousCommands autonomousCommands;
    private Pose2d HomePose_SHORT = new Pose2d(RoadRunnerSubsystem_BLUE.Tile/2, 3 * RoadRunnerSubsystem_BLUE.Tile - 6.93 - 2.56, Math.toRadians(270));
    private Pose2d HomePose_LONG = new Pose2d(1.5 * RoadRunnerSubsystem_BLUE.TileInverted, 3 * RoadRunnerSubsystem_BLUE.TileInverted + (RoadRunnerSubsystem_BLUE.RobotY/2), Math.toRadians(90));

    @Override
    public void initialize() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        RR_Blue = new RoadRunnerCommand_BLUE(drive, HomePose_SHORT, RoadRunnerSubsystem_BLUE.StartingPosition.SHORT,
                RoadRunnerSubsystem_BLUE.Path.INNER, RoadRunnerSubsystem_BLUE.PixelStack.INNER, RoadRunnerSubsystem_BLUE.ParkingPosition.OUTER, telemetry);

        rand = RoadRunnerSubsystem_BLUE.Randomization.LEFT;

        RR_Blue.spikeRandomizationPath(rand);
        RR_Blue.cycle();
        RR_Blue.parking();
        RR_Blue.TrajectoryInit();

        autonomousCommands = new AutonomousCommands(hardwareMap);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        schedule(
                new SequentialCommandGroup(
                        autonomousCommands.spikePixelTake(),
                        new InstantCommand(() -> RR_Blue.runSpike(rand), RR_Blue),
                        autonomousCommands.spikeScoring(),
                        autonomousCommands.randomizationPixelElevator(),
                        new InstantCommand(() -> RR_Blue.runSpike_RandomizedBackdrop(), RR_Blue),
                        autonomousCommands.scoring(),
                        new InstantCommand(() -> RR_Blue.runBackdrop_Station(), RR_Blue),
                        autonomousCommands.stackStationIntake(5),
                        new InstantCommand(() -> RR_Blue.runStation_Backdrop(), RR_Blue),
                        autonomousCommands.elevator(),
                        autonomousCommands.scoring(),
                        new InstantCommand(() -> RR_Blue.runBackdrop_Station(), RR_Blue),
                        autonomousCommands.stackStationIntake(3),
                        new InstantCommand(() -> RR_Blue.runStation_Backdrop(), RR_Blue),
                        autonomousCommands.elevator(),
                        autonomousCommands.scoring(),
                        new InstantCommand(() -> RR_Blue.runParking(), RR_Blue)
                )
        );

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}
