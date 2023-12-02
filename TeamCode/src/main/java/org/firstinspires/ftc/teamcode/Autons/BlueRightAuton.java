package org.firstinspires.ftc.teamcode.Autons;

import com.arcrobotics.ftclib.command.Command;
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

public class BlueRightAuton {

    DriveSubsystem m_driveSS;
    DoubleCassetSubsystem m_casset;
    IntakeSubsystem m_intake;
    LinearSlideSubsystem m_linearSlideSubsystem;
    VisionSubsystem m_vision;

    public BlueRightAuton(DriveSubsystem driveSubsystem,
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

    public Command generate()
    {
        int pathNum = m_vision.getLocation();
        return new WaitCommand(0)
                .andThen(m_driveSS.runTrajectory("BlueLeft/BRGround" + pathNum))
                .andThen(m_intake.outakeCommand())
                .andThen(new WaitCommand(1000))
                .andThen(m_intake.stopCommand())
                .andThen(m_driveSS.runTrajectory("BlueLeft/BRDeposit" + pathNum))
                .andThen(m_linearSlideSubsystem.setAndExtendCommand("LOWLOW"))
                .andThen(new WaitCommand(1000))
                .andThen(m_casset.depositBothCommand())
                .andThen(new WaitCommand(1000))
                .andThen(new ParallelCommandGroup( m_driveSS.runTrajectory("BlueLeft/BRParkLeft"),
                        new SequentialCommandGroup(new WaitCommand(500), m_linearSlideSubsystem.setAndExtendCommand("ZERO")
                        )));

    }
        //.andThen(new RotateCommand(45,0.2, m_driveSS));
    }
