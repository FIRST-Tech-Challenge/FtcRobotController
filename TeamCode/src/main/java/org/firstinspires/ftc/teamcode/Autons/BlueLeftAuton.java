
package org.firstinspires.ftc.teamcode.Autons;

        import com.arcrobotics.ftclib.command.Command;
<<<<<<< Updated upstream
=======
        import com.arcrobotics.ftclib.command.ConditionalCommand;
>>>>>>> Stashed changes
        import com.arcrobotics.ftclib.command.InstantCommand;
        import com.arcrobotics.ftclib.command.ParallelCommandGroup;
        import com.arcrobotics.ftclib.command.SequentialCommandGroup;
        import com.arcrobotics.ftclib.command.WaitCommand;
<<<<<<< Updated upstream

        import org.apache.commons.math3.geometry.euclidean.twod.Line;
        import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
        import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
=======
        import com.arcrobotics.ftclib.command.WaitUntilCommand;

        import org.apache.commons.math3.geometry.euclidean.twod.Line;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.commands.AlignmentScoringCommand;
        import org.firstinspires.ftc.teamcode.commands.GroundDepositCommand;
        import org.firstinspires.ftc.teamcode.commands.RelocalizeOffAprilTags;
        import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
        import org.firstinspires.ftc.teamcode.subsystems.AprilVision;
        import org.firstinspires.ftc.teamcode.subsystems.AprilVision2;
>>>>>>> Stashed changes
        import org.firstinspires.ftc.teamcode.subsystems.CassetSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.DoubleCassetSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.PoseEstimationSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

<<<<<<< Updated upstream
=======

>>>>>>> Stashed changes
public class BlueLeftAuton {

    DriveSubsystem m_driveSS;
    DoubleCassetSubsystem m_casset;
    IntakeSubsystem m_intake;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_vision;
<<<<<<< Updated upstream
=======
    Telemetry m_telemetry;
>>>>>>> Stashed changes

    public BlueLeftAuton(DriveSubsystem driveSubsystem,
                         DoubleCassetSubsystem cassetSubsystem,
                         IntakeSubsystem intakeSubsystem,
                         LinearSlideSubsystem linearSlideSubsystem,
<<<<<<< Updated upstream
                         VisionSubsystem visionSubsystem
=======
                         VisionSubsystem vision, Telemetry telemetry
>>>>>>> Stashed changes
                         )
    {
        m_driveSS = driveSubsystem;
       m_casset = cassetSubsystem;
       m_intake = intakeSubsystem;
       m_linearSlideSubsystem = linearSlideSubsystem;
<<<<<<< Updated upstream
       m_vision = visionSubsystem;
=======
       m_vision = vision;
       m_telemetry = telemetry;
>>>>>>> Stashed changes
    }

    public Command generate()
    {
<<<<<<< Updated upstream
        int pathNum = m_vision.getLocation();
        return new WaitCommand(0)
                .andThen(m_driveSS.runTrajectory("BlueLeft/BLGround" + pathNum))
                .andThen(m_intake.outakeCommand())
                .andThen(new WaitCommand(1000))
                .andThen(m_intake.stopCommand())
                .andThen(m_driveSS.runTrajectory("BlueLeft/BLDeposit" + pathNum))
                .andThen(m_linearSlideSubsystem.setAndExtendCommand("LOWLOW"))
                .andThen(new WaitCommand(1000))
                .andThen(m_casset.depositBothCommand())
                .andThen(new WaitCommand(1000))
                .andThen(new ParallelCommandGroup( m_driveSS.runTrajectory("BlueLeft/BRParkLeft"),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO")
                        )));
=======

        return new WaitCommand(0)
                .andThen(m_intake.stateChangeCommand())
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BLGround0", 40, 40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(new ConditionalCommand(
                        new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BLGround",30,30),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                      new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BLGround2",30,30),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(m_intake.changePoseCommand(0))
                .andThen(new WaitCommand(200))
                .andThen(m_intake.intakeCommand(-0.3))
                .andThen(new WaitCommand(600))
                .andThen(m_intake.stopCommand())
                .andThen(m_intake.stateChangeCommand())
                .andThen(new ConditionalCommand(
                        m_driveSS.runTrajectory("BlueLeft/BLDeposit0",40,40),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 0)))
                .andThen(new ConditionalCommand(
                        m_driveSS.runTrajectory("BlueLeft/BLDeposit0",30,30),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 1)))
                .andThen(new ConditionalCommand(
                        m_driveSS.runTrajectory("BlueLeft/BLDeposit0",30,30),
                        new WaitCommand(0),
                        (()-> m_vision.getLocation() == 2)))
                .andThen(m_linearSlideSubsystem.setAndExtendCommand("auton"))
                .andThen(new WaitCommand(500))
                .andThen(m_casset.depositBothCommand())
                .andThen(new WaitCommand(200))
                .andThen(new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BCycleTo",40,40),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                              m_casset.intakePoseCommand(),
                                new WaitCommand(500)
                                        .andThen(m_intake.changePoseCommand(4))
                                        .andThen(m_intake.stateChangeCommand())
                                        .andThen(m_intake.intakeCommand(1.0))
                        )))
                      .andThen(new WaitCommand(300))
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueLeft/BShimmy"),
                        new SequentialCommandGroup(new WaitCommand(200), m_intake.changePoseCommand(3)
                        ))
                        .andThen(new WaitCommand(1200))
                        .andThen(m_intake.stopCommand())
                          .andThen(m_intake.stateChangeCommand())
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueLeft/BCycleBack",45,45),
                        new SequentialCommandGroup(new WaitCommand(1400),
                                m_linearSlideSubsystem.setAndExtendCommand("LOW"))
                                )))
                        .andThen(new WaitCommand(100))
                        .andThen(m_casset.depositBothCommand())
                        .andThen(new WaitCommand(100))
                .andThen(new ParallelCommandGroup( new TrajectoryFollowerCommand(m_driveSS, "BlueLeft/BCycleTo2",40,40),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                                m_casset.intakePoseCommand(),
                                new WaitCommand(500)
                                        .andThen(m_intake.changePoseCommand(2))
                                        .andThen(m_intake.stateChangeCommand())
                                        .andThen(m_intake.intakeCommand(1.0))
                        )))
                        .andThen(new WaitCommand(300))
                        .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueLeft/BShimmy"),
                                new SequentialCommandGroup(new WaitCommand(200), m_intake.changePoseCommand(1)
                                )))
                .andThen(new WaitCommand(1200))
                .andThen(m_intake.stopCommand())
                .andThen(m_intake.stateChangeCommand())
                .andThen(new WaitCommand(3000))
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueLeft/BCycleBack",45,45),
                        new SequentialCommandGroup(new WaitCommand(1400),
                                m_linearSlideSubsystem.setAndExtendCommand("LOW"))
                ))

                .andThen(new WaitCommand(100))
                .andThen(m_casset.depositBothCommand())
                .andThen(new ParallelCommandGroup(m_driveSS.runTrajectory("BlueRight/BParkLeft"),
                        new SequentialCommandGroup(new WaitCommand(500),
                                m_linearSlideSubsystem.setAndExtendCommand("ZERO"),
                                m_casset.intakePoseCommand())));

>>>>>>> Stashed changes

    }
}