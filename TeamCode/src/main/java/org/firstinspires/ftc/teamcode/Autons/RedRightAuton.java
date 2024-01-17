package org.firstinspires.ftc.teamcode.Autons;

import com.arcrobotics.ftclib.command.Command;
<<<<<<< Updated upstream
=======
import com.arcrobotics.ftclib.command.ConditionalCommand;
>>>>>>> Stashed changes
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
<<<<<<< Updated upstream
import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
=======
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.AprilVision2;
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.subsystems.CassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DoubleCassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RedRightAuton {

    DriveSubsystem m_driveSS;
    DoubleCassetSubsystem m_casset;
    IntakeSubsystem m_intake;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_vision;
<<<<<<< Updated upstream
=======
    Telemetry m_telemetry;
>>>>>>> Stashed changes

    public RedRightAuton(DriveSubsystem driveSubsystem,
                          DoubleCassetSubsystem cassetSubsystem,
                          IntakeSubsystem intakeSubsystem,
                          LinearSlideSubsystem linearSlideSubsystem,
<<<<<<< Updated upstream
                          VisionSubsystem visionSubsystem
=======
                         VisionSubsystem visionSubsystem,
                         Telemetry telemetry
>>>>>>> Stashed changes
    )
    {
        m_driveSS = driveSubsystem;
        m_casset = cassetSubsystem;
        m_intake = intakeSubsystem;
        m_linearSlideSubsystem = linearSlideSubsystem;
        m_vision = visionSubsystem;
<<<<<<< Updated upstream
    }

    public Command generate() {
        int pathNum = m_vision.pathNum;
        return new WaitCommand(0)

                .andThen(m_driveSS.runTrajectory("RedRight/RRGround" + pathNum))
                .andThen(m_intake.outakeCommand())
                .andThen(new WaitCommand(1000))
                .andThen(m_intake.stopCommand())
                .andThen(m_driveSS.runTrajectory("RedRight/RRDeposit" + pathNum))
                .andThen(m_linearSlideSubsystem.setAndExtendCommand("LOWLOW"))
                .andThen(new WaitCommand(1000))
                .andThen(m_casset.depositBothCommand())
                .andThen(new WaitCommand(1000))
                .andThen(new ParallelCommandGroup( m_driveSS.runTrajectory("RedRight/RRParkLeft"),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO")
                                )));


        //.andThen(new RotateCommand(45,0.2, m_driveSS));
=======
        m_telemetry = telemetry;
    }

    public Command generate() {return new WaitCommand(0)
            .andThen(m_intake.stateChangeCommand())
            .andThen(new ConditionalCommand(
                    new TrajectoryFollowerCommand(m_driveSS, "RedRight/RRGround0", 40, 40),
                    new WaitCommand(0),
                    (()-> m_vision.getLocation() == 0)))
            .andThen(new ConditionalCommand(
                    new TrajectoryFollowerCommand(m_driveSS, "RedRight/RRGround0",30,30),
                    new WaitCommand(0),
                    (()-> m_vision.getLocation() == 1)))
            .andThen(new ConditionalCommand(
                    new TrajectoryFollowerCommand(m_driveSS, "RedRight/RRGround0",30,30),
                    new WaitCommand(0),
                    (()-> m_vision.getLocation() == 2)))
            .andThen(m_intake.changePoseCommand(0))
            .andThen(new WaitCommand(200))
            .andThen(m_intake.outakeCommand())
            .andThen(new WaitCommand(300))
            .andThen(m_intake.stopCommand())
            .andThen(m_intake.stateChangeCommand())
            .andThen(new ConditionalCommand(
                    m_driveSS.runTrajectory("RedRight/RRDeposit0",40,40),
                    new WaitCommand(0),
                    (()-> m_vision.getLocation() == 0)))
            .andThen(new ConditionalCommand(
                    m_driveSS.runTrajectory("RedRight/RRDeposit0",30,30),
                    new WaitCommand(0),
                    (()-> m_vision.getLocation() == 1)))
            .andThen(new ConditionalCommand(
                    m_driveSS.runTrajectory("RedRight/RRDeposit0",30,30),
                    new WaitCommand(0),
                    (()-> m_vision.getLocation() == 2)))
            .andThen(m_linearSlideSubsystem.setAndExtendCommand("auton"))
            .andThen(new WaitCommand(500))
            .andThen(m_casset.depositBothCommand())
            .andThen(new WaitCommand(200))
            .andThen(new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "RedRight/RCycleTo",40,40),
                    new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                            m_casset.intakePoseCommand(),
                            new WaitCommand(500)
                                    .andThen(m_intake.changePoseCommand(4))
                                    .andThen(m_intake.stateChangeCommand())
                                    .andThen(m_intake.intakeCommand(1.0))
                    )))
            .andThen(new WaitCommand(300))
            .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedRight/RShimmy"),
                    new SequentialCommandGroup(new WaitCommand(200), m_intake.changePoseCommand(3)
                    ))
                    .andThen(new WaitCommand(1200))
                    .andThen(m_intake.stopCommand())
                    .andThen(m_intake.stateChangeCommand())
                    .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedRight/RCycleBack",45,45),
                            new SequentialCommandGroup(new WaitCommand(1400),
                                    m_linearSlideSubsystem.setAndExtendCommand("LOW"))
                    )))
            .andThen(new WaitCommand(100))
            .andThen(m_casset.depositBothCommand())
            .andThen(new WaitCommand(100))
            .andThen(new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "RedRight/RCycleTo2",40,40),
                    new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                            m_casset.intakePoseCommand(),
                            new WaitCommand(500)
                                    .andThen(m_intake.changePoseCommand(2))
                                    .andThen(m_intake.stateChangeCommand())
                                    .andThen(m_intake.intakeCommand(1.0))
                    )))
            .andThen(new WaitCommand(300))
            .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedRight/RShimmy"),
                    new SequentialCommandGroup(new WaitCommand(200), m_intake.changePoseCommand(1)
                    )))
            .andThen(new WaitCommand(1200))
            .andThen(m_intake.stopCommand())
            .andThen(m_intake.stateChangeCommand())
            .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedRight/RCycleBack",45,45),
                    new SequentialCommandGroup(new WaitCommand(1400),
                            m_linearSlideSubsystem.setAndExtendCommand("LOW"))
            ))

            .andThen(new WaitCommand(100))
            .andThen(m_casset.depositBothCommand())
                        .andThen(new WaitCommand(400))
        .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedRight/RParkLeft"),
                new SequentialCommandGroup(new WaitCommand(500),
                        m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                        m_casset.intakePoseCommand())));


>>>>>>> Stashed changes
    }
}