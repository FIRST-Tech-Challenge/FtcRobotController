//package org.firstinspires.ftc.teamcode.commands;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.robotContainer.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.ClawIntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;
//
//import java.util.function.DoubleSupplier;
//
//@Config
//public class ScoringCommand extends CommandBase {
//    private final ClawIntakeSubsystem m_clawSubsystem;
//    private final LinearSlideSubsystem m_linearSlideSubsystem;
//    private int m_prevSetPoint;
//    private double m_prevPower;
//    public static double m_powerScaling = 0.5;
//
//    private final DoubleSupplier m_leftTriggerSupplier;
//
//    public ScoringCommand(ClawIntakeSubsystem clawSubsystem,
//                          LinearSlideSubsystem slideSubsystem,
//                          DoubleSupplier leftTriggerSupplier)
//    {
//        m_clawSubsystem = clawSubsystem;
//        m_linearSlideSubsystem = slideSubsystem;
//        m_leftTriggerSupplier = leftTriggerSupplier;
//        addRequirements(m_clawSubsystem, m_linearSlideSubsystem);
//    }
//
//    @Override
//    public void initialize()
//    {
//        m_prevSetPoint= m_linearSlideSubsystem.getPositionSetpoint();
//        m_prevPower = m_linearSlideSubsystem.getElevatorPower();
//        m_linearSlideSubsystem.stateTransition(Constants.LinearSlideState.SCORING);
//        m_linearSlideSubsystem.setElevatorPower(m_leftTriggerSupplier.getAsDouble() * m_powerScaling);
//    }
//
//    @Override
//    public void execute()
//    {
//        m_linearSlideSubsystem.setElevatorPower(m_leftTriggerSupplier.getAsDouble() * m_powerScaling);
//    }
//
//    @Override
//    public boolean isFinished()
//    {
//        return Thread.currentThread().isInterrupted() || !m_linearSlideSubsystem.isLinearSlideBusy();
//    }
//
//    @Override
//    public void end(boolean interrupted)
//    {
//        m_clawSubsystem.open();
//        m_linearSlideSubsystem.setElevatorPower(0);
//        m_linearSlideSubsystem.setPositionSetPoint(m_prevSetPoint);
//    }
//}
