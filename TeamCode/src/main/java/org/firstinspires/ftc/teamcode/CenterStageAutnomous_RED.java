package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "CenterStageAutonomous_RED", group = "Final Autonomous")
public class CenterStageAutnomous_RED extends CommandOpMode {

    private OuttakeSusystem outtakeSusystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ElevatorCommand elevatorCommand;
    private IntakeArmSubsystem intakeArmSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private SampleMecanumDrive drive;
    private RoadRunnerCommand_RED RR_Red;
    private RoadRunnerSubsystem_RED.Randomization rand;

    private Pose2d HomePose_SHORT = new Pose2d(RoadRunnerSubsystem_RED.Tile/2, 3 * RoadRunnerSubsystem_RED.TileInverted + 6.93 + 2.56, Math.toRadians(90));
    private Pose2d HomePose_LONG = new Pose2d(1.5 * RoadRunnerSubsystem_RED.TileInverted, 3 * RoadRunnerSubsystem_RED.TileInverted + (RoadRunnerSubsystem_RED.RobotY/2), Math.toRadians(90));

    private SequentialCommandGroup temp;
    public SequentialCommandGroup randomizationPixelElevator(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO),
                new InstantCommand(outtakeSusystem::go_outtake_first),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup elevator(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOW),
                new InstantCommand(outtakeSusystem::go_outtake_first),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup scoring_randomization(){
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::wheel_release),
                new WaitCommand(800),
                new InstantCommand(outtakeSusystem::wheel_stop)
        );
    }

    public SequentialCommandGroup scoring(){
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::wheel_release),
                new WaitCommand(1300),
                new InstantCommand(outtakeSusystem::wheel_stop)
        );
    }

    public SequentialCommandGroup resetElevator() {
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::go_intake_second),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_intake_first),
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING)
        );
    }

    public SequentialCommandGroup stackStationIntake(int index) {
        return new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::run),
                new InstantCommand(outtakeSusystem::wheel_grab),
                new WaitCommand(150),
                new SequentialCommandGroup(
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(index)),
                        new WaitCommand(500),
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(index - 1)),
                        new WaitCommand(800)
                ),
                new ParallelCommandGroup(
                        new InstantCommand(intakeSubsystem::stop),
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(6)),
                        new InstantCommand(outtakeSusystem::wheel_stop)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(intakeSubsystem::reverse),
                        new WaitCommand(600),
                        new InstantCommand(intakeSubsystem::stop)
                )
        );
    }

    @Override
    public void initialize() {
        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> 0, outtakeSusystem);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        RR_Red = new RoadRunnerCommand_RED(drive, HomePose_SHORT, RoadRunnerSubsystem_RED.StartingPosition.SHORT,
                RoadRunnerSubsystem_RED.Path.INNER, RoadRunnerSubsystem_RED.PixelStack.INNER,
                RoadRunnerSubsystem_RED.ParkingPosition.INNER, telemetry);

        rand = RoadRunnerSubsystem_RED.Randomization.LEFT;

        RR_Red.spikeRandomizationPath(rand);
        RR_Red.cycle();
        RR_Red.parking();
        RR_Red.TrajectoryInit();
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        // SPIKE
        new InstantCommand(intakeArmSubsystem::lockPixel, intakeArmSubsystem).schedule();
        RR_Red.runSpike(rand);
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        // BACKDROP - YELLOW
        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem).schedule();
        randomizationPixelElevator().schedule();
        RR_Red.runSpike_RandomizedBackdrop();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = scoring_randomization();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }
        // STACK FIRST 2
        temp = new SequentialCommandGroup(
                new WaitCommand(600),
                resetElevator()
        );
        temp.schedule();
        RR_Red.runBackdrop_Station_First_Cycle();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = stackStationIntake(5);
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = new SequentialCommandGroup(
                new WaitCommand(1800),
                elevator()
        );

        temp.schedule();

        RR_Red.runStation_Backdrop_First_Cycle();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = scoring();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        // STACK Second 2
        temp = new SequentialCommandGroup(
                new WaitCommand(600),
                resetElevator()
        );
        temp.schedule();

        RR_Red.runBackdrop_Station_Second_Cycle();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = stackStationIntake(3);
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = new SequentialCommandGroup(
                new WaitCommand(1800),
                elevator()
        );

        temp.schedule();

        RR_Red.runStation_Backdrop_Second_Cycle();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = scoring();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }


        temp = resetElevator();
        temp.schedule();
        RR_Red.runParking();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        reset();
    }
}
