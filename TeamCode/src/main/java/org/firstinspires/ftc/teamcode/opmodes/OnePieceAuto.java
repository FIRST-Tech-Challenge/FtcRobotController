package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ElevatorGoTo;
import org.firstinspires.ftc.teamcode.commands.Move;
import org.firstinspires.ftc.teamcode.commands.ScoreAtBucket;
import org.firstinspires.ftc.teamcode.commands.SetArmPosition;
import org.firstinspires.ftc.teamcode.commands.Strafe;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

@Autonomous
public class OnePieceAuto extends CommandOpMode {

    private Drivetrain drivetrain;
    private Arm arm;
    private Elevator elevator;


    @Override
    public void initialize() {
        this.drivetrain = new Drivetrain(hardwareMap, new Pose2d(-47,-58.5, Math.toRadians(45)), telemetry);
        this.arm = new Arm(hardwareMap);
        this.elevator = new Elevator(hardwareMap,telemetry);

        register(drivetrain, arm, elevator);
        waitForStart();

        schedule(
                new SequentialCommandGroup(
                        new Strafe(drivetrain, -0.25, 1500),
                        new Move(drivetrain, 0.25, 400),
                        new ElevatorGoTo(elevator, 38),
                        new SetArmPosition(arm, Arm.ArmState.SCORE).withTimeout(1500),
                        new ParallelCommandGroup(
                                new ElevatorGoTo(elevator, 0),
                                new SetArmPosition(arm, Arm.ArmState.INTAKE).withTimeout(500)
                        )
                )

        );

    }
}
