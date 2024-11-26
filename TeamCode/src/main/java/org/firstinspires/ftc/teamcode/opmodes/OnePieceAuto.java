package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ScoreAtBucket;
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
                new ScoreAtBucket(this.drivetrain, this.arm, this.elevator)
        );

    }
}
