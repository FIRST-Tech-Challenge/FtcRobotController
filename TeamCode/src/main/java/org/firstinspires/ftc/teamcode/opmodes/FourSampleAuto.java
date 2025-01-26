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

import org.firstinspires.ftc.teamcode.commands.CreepIntake;
import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.ExtendIntake;
import org.firstinspires.ftc.teamcode.commands.ExtendIntakeVariable;
import org.firstinspires.ftc.teamcode.commands.PivotIntake;
import org.firstinspires.ftc.teamcode.commands.RetractIntake;
import org.firstinspires.ftc.teamcode.commands.SetArmPosition;
import org.firstinspires.ftc.teamcode.commands.SetRollerState;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeRoller;

@Autonomous(name = "Four Sample Auto")
public class FourSampleAuto extends CommandOpMode {

    private Drivetrain drivetrain;
    private Claw claw;
    private Elevator elevator;
    private Intake intake;
    private Arm arm;
    private IntakeRoller roller;

    @Override
    public void initialize() {

        drivetrain = new Drivetrain(hardwareMap, new Pose2d(-38, -61, Math.toRadians(90)), telemetry);
        claw = new Claw(hardwareMap);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        arm = new Arm(hardwareMap);
        roller = new IntakeRoller(hardwareMap);

        Vector2d homePosition = new Vector2d(-59, -56);

        Action depositLoadSample = drivetrain.getTrajectoryBuilder(new Pose2d(-38, -61, Math.toRadians(90)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();

        Action pickUpFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d(-59,-56, Math.toRadians(45)))
            .strafeToLinearHeading(new Vector2d(-31, -35), Math.toRadians(150))
            .build();

        Action depositFirstSample = drivetrain.getTrajectoryBuilder(new Pose2d(-31, -35, Math.toRadians(150)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();


        Action pickUpSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(-59, -56, Math.toRadians(45)))
            .strafeToLinearHeading(new Vector2d(-40, -35), Math.toRadians(150))
            .build();

        Action depositSecondSample = drivetrain.getTrajectoryBuilder(new Pose2d(-40, -35, Math.toRadians(150)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();
        Action pickUpThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(-59, -56, Math.toRadians(45)))
        .strafeToLinearHeading(new Vector2d(-48, -25), Math.toRadians(180))
            .build();
        Action depositThirdSample = drivetrain.getTrajectoryBuilder(new Pose2d(-48, -25, Math.toRadians(180)))
            .strafeToLinearHeading(homePosition, Math.toRadians(45))
            .build();




        schedule(new RunCommand(() -> telemetry.update()));
        register(drivetrain, claw, elevator, intake, roller);
        waitForStart();

        schedule(new SequentialCommandGroup(
            new PivotIntake(Intake.IntakeState.HOME, intake),
            new RetractIntake(intake),
            new ParallelCommandGroup(
                new TrajectoryCommand(depositLoadSample, drivetrain),
                new ElevatorGoTo(elevator, 35).withTimeout(2500)
            ),
            new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1500),
            new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(250),
            new ParallelCommandGroup(
                new ElevatorGoTo(elevator,0).withTimeout(1500),
                new TrajectoryCommand(pickUpFirstSample, drivetrain)
            ),
            new SequentialCommandGroup(
                new ExtendIntakeVariable(intake, ()->0.15).withTimeout(500),
                new PivotIntake(Intake.IntakeState.COLLECT,intake)
            ),
//            new WaitCommand(1000),
            new ParallelCommandGroup(
                new SetRollerState(roller, IntakeRoller.States.INTAKE),
                new CreepIntake(intake, 0.73, 1, telemetry)
            ),
            new WaitCommand(1000),
            new ParallelCommandGroup(
                new SetRollerState(roller, IntakeRoller.States.STOP),
                new PivotIntake(Intake.IntakeState.STORE, intake)
            ),
            new RetractIntake(intake),
            new WaitCommand(1000),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    // depositing
                    new SetRollerState(roller, IntakeRoller.States.OUTTAKE),
                    new WaitCommand(750),
                    new PivotIntake(Intake.IntakeState.HOME, intake),
                    new ElevatorGoTo(elevator, 35)
                ),
                new TrajectoryCommand(depositFirstSample, drivetrain)
            ),
            new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1500),
            new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(250),
            new ElevatorGoTo(elevator, 0).withTimeout(2000),
            new TrajectoryCommand(pickUpSecondSample, drivetrain),
            new SequentialCommandGroup(
                new ExtendIntakeVariable(intake, ()->0.15).withTimeout(500),
                new PivotIntake(Intake.IntakeState.COLLECT,intake)
            ),
//            new WaitCommand(1000),
            new ParallelCommandGroup(
                new SetRollerState(roller, IntakeRoller.States.INTAKE),
                new CreepIntake(intake, 0.7, 1.5, telemetry)
            ),
            new WaitCommand(1000),
            new ParallelCommandGroup(
                new SetRollerState(roller, IntakeRoller.States.STOP),
                new PivotIntake(Intake.IntakeState.STORE, intake)
            ),
            new RetractIntake(intake),
            new WaitCommand(500),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    // depositing
                    new SetRollerState(roller, IntakeRoller.States.OUTTAKE),
                    new WaitCommand(750),
                    new PivotIntake(Intake.IntakeState.HOME, intake),
                    new ElevatorGoTo(elevator, 35).withTimeout(2000)
                ),
                new TrajectoryCommand(depositSecondSample, drivetrain)
            ),
            new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1500),
            new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(250),
            new ElevatorGoTo(elevator, 0).withTimeout(2000),
            new TrajectoryCommand(pickUpThirdSample, drivetrain),
            new SequentialCommandGroup(
                new ExtendIntakeVariable(intake, ()->0.15).withTimeout(500),
                new PivotIntake(Intake.IntakeState.COLLECT,intake)
            ),
//            new WaitCommand(1000),
            new ParallelCommandGroup(
                new SetRollerState(roller, IntakeRoller.States.INTAKE),
                new CreepIntake(intake, 0.7, 1.5, telemetry)
            ),
            new WaitCommand(500),
            new ParallelCommandGroup(
                new SetRollerState(roller, IntakeRoller.States.STOP),
                new PivotIntake(Intake.IntakeState.STORE, intake)
            ),
            new RetractIntake(intake),
            new WaitCommand(500),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    // depositing
                    new SetRollerState(roller, IntakeRoller.States.OUTTAKE),
                    new WaitCommand(750),
                    new PivotIntake(Intake.IntakeState.HOME, intake),
                    new ElevatorGoTo(elevator, 35).withTimeout(2000)
                ),
                new TrajectoryCommand(depositThirdSample, drivetrain)
            ),
            new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1500),
            new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(250),
            new ElevatorGoTo(elevator, 0).withTimeout(2000)




            ));
    }
}
