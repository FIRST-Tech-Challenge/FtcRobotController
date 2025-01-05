package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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

@Autonomous(name = "2 Specimen Auto")
public class TwoSpecimenAuto extends CommandOpMode {
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
                .strafeTo(new Vector2d(0, -30.5))
                        .build();

        Action driveToPrimePickup = drivetrain.getTrajectoryBuilder(new Pose2d(0, -30.5, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(43, -55), Math.toRadians(0))
                .build();

        Action driveToPickup = drivetrain.getTrajectoryBuilder(new Pose2d(43, -55, Math.toRadians(0)))
                .strafeTo(new Vector2d(43, -65)).build();
        register(drivetrain, claw, elevator, intake);

        Action driveToScore2 = drivetrain.getTrajectoryBuilder(new Pose2d(43, -63, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-6, -30.5), Math.toRadians(180)).build();
        schedule(new RunCommand(() -> telemetry.update()));
        waitForStart();

        schedule(new SequentialCommandGroup(
                new RetractIntake(intake),
                new PivotIntake(Intake.IntakeState.HOME, intake),
                new CloseClaw(claw),
                // drive to chamber and raise elevator scoring height
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToFirstScore, drivetrain),
                        new ElevatorGoTo(elevator, 28.5)
                ),
                new ElevatorGoTo(elevator, 20.5),
                new OpenClaw(claw),
                // lower elevator to 0, prime to pick up specimen
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToPrimePickup, drivetrain),
                        new ElevatorGoTo(elevator, 0).withTimeout(3000)
                ),
                // pick up specimen and close claw, then raise elevator
                new TrajectoryCommand(driveToPickup, drivetrain),
                new CloseClaw(claw),
                new ElevatorGoTo(elevator, 6).withTimeout(3000),
                // drive to chamber and raise elevator to scoring height
                new ParallelCommandGroup(
                        new TrajectoryCommand(driveToScore2, drivetrain),
                        new ElevatorGoTo(elevator, 28.5).withTimeout(3000)
                ),
                new ElevatorGoTo(elevator, 18).withTimeout(3000),
                new OpenClaw(claw)
        ));
    }
}
