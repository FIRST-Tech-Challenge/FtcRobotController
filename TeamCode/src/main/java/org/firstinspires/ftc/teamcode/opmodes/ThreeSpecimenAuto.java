package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "3 Specimen Auto")
public class ThreeSpecimenAuto extends CommandOpMode {
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

        Vector2d primePickup = new Vector2d(43, -55);
        Vector2d pickup = new Vector2d(43, -65);

        Action driveToFirstScore = drivetrain.getTrajectoryBuilder(new Pose2d(0, -61, Math.toRadians(180)))
                .strafeTo(new Vector2d(0, -31))
                .build();

        Action driveToPrimePush = drivetrain.getTrajectoryBuilder(new Pose2d(0, -31, Math.toRadians(180)))
                .strafeTo(new Vector2d(0, -44))
                .strafeToLinearHeading(new Vector2d(31, -44), Math.toRadians(0))
                .strafeTo(new Vector2d(40, -15))
                .build();

        Action pushSpecimen = drivetrain.getTrajectoryBuilder(new Pose2d(40, -15, Math.toRadians(0)))
                .strafeTo(new Vector2d(47, -15))
                .strafeTo(new Vector2d(47, -55))
//                .strafeTo(new Vector2d(47, -15))
//                .strafeTo(new Vector2d(58, -15))
//                .strafeTo(new Vector2d(58, -55))
                .build();

        Action driveToPrimePickup = drivetrain.getTrajectoryBuilder(new Pose2d(47, -55, Math.toRadians(0)))
                .strafeTo(new Vector2d(47, -47))
                .build();

        Action driveToPrimePickup2 = drivetrain.getTrajectoryBuilder(new Pose2d(-6, -31, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(47, -62), Math.toRadians(0))
                .strafeTo(new Vector2d(47, -65))
                .build();

        Action driveToPickup = drivetrain.getTrajectoryBuilder(new Pose2d(47, -47, Math.toRadians(0)))
                .strafeTo(new Vector2d(47, -65)).build();

        Action driveToScore2 = drivetrain.getTrajectoryBuilder(new Pose2d(47, -65, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-6, -31), Math.toRadians(180)).build();

        Action driveToScore3 = drivetrain.getTrajectoryBuilder(new Pose2d(47, -65, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(3, -31), Math.toRadians(180)).build();

        Action driveToPark = drivetrain.getTrajectoryBuilder(new Pose2d(3, -31, Math.toRadians(180)))
                        .strafeTo(new Vector2d(47, -60)).build();

        schedule(new RunCommand(() -> telemetry.update()));
        register(drivetrain, claw, elevator, intake);
        waitForStart();

        schedule(new SequentialCommandGroup(
                new RetractIntake(intake),
                new PivotIntake(Intake.IntakeState.HOME, intake),
                new CloseClaw(claw),
                // drive to chamber and raise elevator scoring height
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToFirstScore, drivetrain),
                        new ElevatorGoTo(elevator, 27).withTimeout(2000)
                ),
                new ElevatorGoTo(elevator, 20.5),
                new OpenClaw(claw),
                // drive to start push
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToPrimePush, drivetrain),
                        new ElevatorGoTo(elevator, 0).withTimeout(2000)
                ),
                new TrajectoryCommand(pushSpecimen, drivetrain),
                new TrajectoryCommand(driveToPrimePickup, drivetrain),
                new WaitCommand(500),
                new TrajectoryCommand(driveToPickup, drivetrain),
                new CloseClaw(claw),
                new WaitCommand(250),
                new ElevatorGoTo(elevator, 6).withTimeout(3000),
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToScore2, drivetrain),
                        new ElevatorGoTo(elevator, 27).withTimeout(3000)
                ),
                new ElevatorGoTo(elevator, 18).withTimeout(3000),
                new OpenClaw(claw),
//                // pick up 3
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToPrimePickup2, drivetrain),
                        new ElevatorGoTo(elevator, 0).withTimeout(2000)
                ),
//                new TrajectoryCommand(driveToPickup, drivetrain),
                new CloseClaw(claw),
                new WaitCommand(250),
//                // score 3
                new ElevatorGoTo(elevator, 6).withTimeout(3000),
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToScore3, drivetrain),
                        new ElevatorGoTo(elevator, 28.5).withTimeout(3000)
                ),
                new ElevatorGoTo(elevator, 18).withTimeout(3000),
                new OpenClaw(claw),
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToPark, drivetrain),
                        new ElevatorGoTo(elevator, 0).withTimeout(2000)
                )
//                // pick up 4 - we don't do this
//                new ParallelCommandGroup(
//                        new StrafeWithHeading(primePickup,0, drivetrain),
//                        new ElevatorGoTo(elevator, 0).withTimeout(2000),
//                        ),
//                new StrafeToPos(pickup, drivetrain),
//                new CloseClaw(claw),
//                // score 4
//                new ElevatorGoTo(elevator, 6).withTimeout(3000),
//                new ParallelCommandGroup(
//                        new StrafeWithHeading(new Vector2d(-18, -30.5), Math.toRadians(180), drivetrain),
//                        new ElevatorGoTo(elevator, 28.5).withTimeout(3000)
//                ),
//                new ElevatorGoTo(elevator, 18).withTimeout(3000),
//                new OpenClaw(claw)
        ));
    }
}
