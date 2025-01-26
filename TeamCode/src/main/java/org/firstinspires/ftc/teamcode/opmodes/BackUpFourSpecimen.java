package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.ExtendIntake;
import org.firstinspires.ftc.teamcode.commands.ExtendIntakeVariable;
import org.firstinspires.ftc.teamcode.commands.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "BackUp Four Specimen Auto")
public class BackUpFourSpecimen extends CommandOpMode {

    private Drivetrain drivetrain;
    private Claw claw;
    private Elevator elevator;
    private Intake intake;

    @Override
    public void initialize() {
        drivetrain = new Drivetrain(hardwareMap, new Pose2d(0, -61, Math.toRadians(180)), telemetry);
        claw = new Claw(hardwareMap);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        Action driveToFirstScore = drivetrain.getTrajectoryBuilder(new Pose2d(0, -61, Math.toRadians(180)))
            .strafeTo(new Vector2d(0, -31))
            .build();

        Action driveToPrimePush = drivetrain.getTrajectoryBuilder(new Pose2d(0, -31, Math.toRadians(180)))
            .splineToConstantHeading(new Vector2d(0,-39), Math.toRadians(0))
            .splineToSplineHeading(new Pose2d(24.75,-37, Math.toRadians(50)), Math.toRadians(50))
            .build();

        Action pushFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d(24.75, -37, Math.toRadians(50)))
            .strafeToLinearHeading(new Vector2d(24.75,-50), Math.toRadians(330))
            .build();

        Action goToSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(24.75,-50, Math.toRadians(330)))
            .strafeToLinearHeading(new Vector2d(32.5, -37), Math.toRadians(35))
            .build();

        Action pushSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(32.5,-37, Math.toRadians(35)))
            .strafeToLinearHeading(new Vector2d(32.5, -50), Math.toRadians(330))
            .build();

        Action goToThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(32.5,-50, Math.toRadians(330)))
            .strafeToLinearHeading(new Vector2d(41, -20), Math.toRadians(0))
            .build();

        Action pushThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(41,-20, Math.toRadians(0)))
            .strafeToLinearHeading(new Vector2d(37, -67.5), Math.toRadians(0))
            .build();

        Action scoreSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(38, -66, Math.toRadians(0)))
            .strafeToSplineHeading(new Vector2d(3, -31), Math.toRadians(180))
            .build();

        Action pickUpThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(3, -31, Math.toRadians(180)))
            .strafeToSplineHeading(new Vector2d(39, -65.5), Math.toRadians(0))
            .build();

        Action scoreThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(39, -65.5, Math.toRadians(0)))
            .strafeToSplineHeading(new Vector2d(-2, -31), Math.toRadians(180))
            .build();

        Action pickUpFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d(-2, -31, Math.toRadians(180)))
            .strafeToSplineHeading(new Vector2d(39, -66), Math.toRadians(0))
            .build();

        Action scoreFourthSample = drivetrain.getTrajectoryBuilder(new Pose2d(39, -66, Math.toRadians(0)))
            .strafeToSplineHeading(new Vector2d(-6, -31), Math.toRadians(180))
            .build();


        Action driveToPark = drivetrain.getTrajectoryBuilder(new Pose2d(-6, -31, Math.toRadians(180)))
            .strafeTo(new Vector2d(47, -60)).build();

        schedule(new RunCommand(() -> telemetry.update()));
        register(drivetrain, claw, elevator, intake);
        waitForStart();

        schedule(new SequentialCommandGroup(
            new RetractIntake(intake),
            new PivotIntake(Intake.IntakeState.HOME, intake),
            new CloseClaw(claw),
            new ParallelCommandGroup(
                new TrajectoryCommand(driveToFirstScore, drivetrain),
                new ElevatorGoTo(elevator, 25).withTimeout(2000)
            ),
            new ElevatorGoTo(elevator, 18),
            new OpenClaw(claw),
            new ParallelCommandGroup(
                new TrajectoryCommand(driveToPrimePush, drivetrain),
                new ElevatorGoTo(elevator, 0).withTimeout(2000)
            ),
            new ExtendIntake(intake),
            new PivotIntake(Intake.IntakeState.COLLECT, intake),
            new WaitCommand(500),
            new TrajectoryCommand(pushFirstSample, drivetrain),
            new ParallelCommandGroup(
                //new ExtendIntakeVariable(intake, () -> 0.3).withTimeout(300),
                new PivotIntake(Intake.IntakeState.HOME, intake),
                new TrajectoryCommand(goToSecondSample, drivetrain)
            ),
//            new ExtendIntake(intake),
            new PivotIntake(Intake.IntakeState.COLLECT, intake),
            new WaitCommand(250),
            new TrajectoryCommand(pushSecondSample, drivetrain),
            new ParallelCommandGroup(
//                new ExtendIntakeVariable(intake, () -> 0.3).withTimeout(300),
                new PivotIntake(Intake.IntakeState.HOME, intake),
                new TrajectoryCommand(goToThirdSample, drivetrain)
            ),
//            new ExtendIntake(intake),
//            new PivotIntake(Intake.IntakeState.COLLECT, intake),
            //new WaitCommand(100),
            new ParallelRaceGroup(
                new TrajectoryCommand(pushThirdSample, drivetrain),
                new WaitCommand(500).andThen(new ExtendIntakeVariable(intake, () -> 0.95))
            ),
            new PivotIntake(Intake.IntakeState.HOME, intake),
            new RetractIntake(intake),
            new CloseClaw(claw),
            new WaitCommand(500),
            new ElevatorGoTo(elevator, 6).withTimeout(1000), // pick up 2nd
            new ParallelCommandGroup(
                new ElevatorGoTo(elevator, 25).withTimeout(2000),
                new TrajectoryCommand(scoreSecondSample, drivetrain)
            ),
            new ElevatorGoTo(elevator, 18).withTimeout(2000),
            new OpenClaw(claw), // Scored second sample
            new ParallelCommandGroup(
                new ElevatorGoTo(elevator, 0 ).withTimeout(2000),
                new TrajectoryCommand(pickUpThirdSample, drivetrain)
            ),
            new CloseClaw(claw),
            new WaitCommand(250),
            new ParallelCommandGroup(
                new ElevatorGoTo(elevator, 25).withTimeout(2000),
                new TrajectoryCommand(scoreThirdSample, drivetrain)
            ),
            new ElevatorGoTo(elevator, 18).withTimeout(2000),
            new OpenClaw(claw),
            new ParallelCommandGroup(
                new ElevatorGoTo(elevator, 0 ).withTimeout(2000),
                new TrajectoryCommand(pickUpFourthSample, drivetrain)
            ),
            new CloseClaw(claw),
            new WaitCommand(250),
            new ParallelCommandGroup(
                new ElevatorGoTo(elevator, 25).withTimeout(2000),
                new TrajectoryCommand(scoreFourthSample, drivetrain)
            ),
            new ElevatorGoTo(elevator, 18).withTimeout(2000),
            new OpenClaw(claw),
            new ParallelCommandGroup(
                new ElevatorGoTo(elevator, 0).withTimeout(1000),
                new TrajectoryCommand(driveToPark, drivetrain)
            )
        ));
    }
}
