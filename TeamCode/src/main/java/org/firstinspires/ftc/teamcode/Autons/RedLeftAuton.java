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
import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.CassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DoubleCassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class RedLeftAuton {

    DriveSubsystem m_driveSS;
    DoubleCassetSubsystem m_casset;
    IntakeSubsystem m_intake;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_vision;

    public RedLeftAuton(DriveSubsystem driveSubsystem,
                          DoubleCassetSubsystem cassetSubsystem,
                          IntakeSubsystem intakeSubsystem,
                          LinearSlideSubsystem linearSlideSubsystem,
                          VisionSubsystem visionSubsystem
    )
    {
        m_driveSS = driveSubsystem;
        m_casset = cassetSubsystem;
        m_intake = intakeSubsystem;
        m_linearSlideSubsystem = linearSlideSubsystem;
        m_vision = visionSubsystem;
    }

    public Command generate() {
<<<<<<< Updated upstream
        int pathNum = m_vision.getLocation();
        return new WaitCommand(0)
                .andThen(m_driveSS.runTrajectory("RedRight/BRGround" + pathNum))
                .andThen(m_intake.outakeCommand())
                .andThen(new WaitCommand(1000))
                .andThen(m_intake.stopCommand())
                .andThen(m_driveSS.runTrajectory("RedRight/BRDeposit" + pathNum))
                .andThen(m_linearSlideSubsystem.setAndExtendCommand("LOWLOW"))
                .andThen(new WaitCommand(1000))
                .andThen(m_casset.depositBothCommand())
                .andThen(new WaitCommand(1000))
                .andThen(new ParallelCommandGroup( m_driveSS.runTrajectory("RedRight/RRParkLeft"),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO")
                        )));


        //.andThen(new RotateCommand(45,0.2, m_driveSS));
=======


        return new WaitCommand(0)
                .andThen(new WaitCommand(0))
                .andThen(m_intake.stateChangeCommand())
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "RedLeft/RLGround0", 40, 40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "RedLeft/RLGround1",40,40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "RedLeft/RLGround2",40,40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(m_intake.changePoseCommand(0))
                .andThen(new WaitCommand(200))
                .andThen(m_intake.intakeCommand(-0.5))
                .andThen(new WaitCommand(3000))
                .andThen(m_intake.stopCommand())
                .andThen(m_intake.stateChangeCommand())
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup(m_driveSS.runTrajectory("RedLeft/RLToStack0",40,40),
                                new SequentialCommandGroup(m_intake.intakeCommand(1.0),
                                        m_intake.changePoseCommand(4),
                                        m_intake.stateChangeCommand())),

                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup(m_driveSS.runTrajectory("RedLeft/RLToStack1",40,40),
                                new SequentialCommandGroup(m_intake.intakeCommand(1.0),
                                        m_intake.changePoseCommand(4),
                                        m_intake.stateChangeCommand())),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))

                .andThen(new ConditionalCommand(

                        new ParallelCommandGroup(m_driveSS.runTrajectory("RedLeft/RLToStack2",40,40),
                                new SequentialCommandGroup(m_intake.intakeCommand(1.0),
                                        m_intake.changePoseCommand(4),
                                        m_intake.stateChangeCommand())),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))

                .andThen(new WaitCommand(700))
                .andThen(m_intake.stopCommand())
                .andThen(m_intake.setStateCommand("stow"))
                .andThen(new WaitCommand(200))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "RedLeft/RLCycleTo0",40,40),
                                new SequentialCommandGroup(new WaitCommand(3500), m_linearSlideSubsystem.setAndExtendCommand("auton")
                                )),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "RedLeft/RLCycleTo1",40,40),
                                new SequentialCommandGroup(new WaitCommand(3500), m_linearSlideSubsystem.setAndExtendCommand("auton")
                                )),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                        new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "RedLeft/RLCycleTo2",40,40),
                                new SequentialCommandGroup(new WaitCommand(3500), m_linearSlideSubsystem.setAndExtendCommand("auton")
                                )),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))

                .andThen(new WaitCommand(300))
                .andThen(m_casset.depositBothCommand())
                .andThen(new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "RedLeft/RLCycleBack",40,40),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                                m_casset.intakePoseCommand(),
                                new WaitCommand(1500)
                                        .andThen(m_intake.changePoseCommand(3))
                                        .andThen(m_intake.setStateCommand("intake"))
                                        .andThen(m_intake.intakeCommand(1.0))
                        )))
                .andThen(new WaitCommand(300))
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedLeft/RShimmy"),
                        new SequentialCommandGroup(new WaitCommand(200), m_intake.changePoseCommand(2)
                        )))
                .andThen(new WaitCommand(1200))
                .andThen(m_intake.stopCommand())

                .andThen(m_intake.stateChangeCommand())
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedLeft/RLCycleTo1",45,45),
                        new SequentialCommandGroup(new WaitCommand(3500),
                                m_linearSlideSubsystem.setAndExtendCommand("LOW"))
                ))

                .andThen(new WaitCommand(100))
                .andThen(m_casset.depositBothCommand())


                .andThen(new WaitCommand(400))
               .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("RedLeft/RParkLeft"),
                        new SequentialCommandGroup(new WaitCommand(500),
                                m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                                m_casset.intakePoseCommand())));

>>>>>>> Stashed changes
    }
}