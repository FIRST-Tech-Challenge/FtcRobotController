//package org.firstinspires.ftc.teamcode.commands;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.AlignmentSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//
//import java.util.function.DoubleSupplier;
//
//@Config
//public class AlignmentCommand extends CommandBase {
//    public static double p = 0.4;
//    public static double i = 0.0;
//    public static double d = 0.05;
//    private final AlignmentSubsystem m_alignmentSubsystem;
//    private final DriveSubsystem m_driveSubsystem;
//    private final DoubleSupplier m_leftYSupplier;
//    private final DoubleSupplier m_leftXSupplier;
//    private final DoubleSupplier m_rightXSupplier;
//    private final DoubleSupplier m_rightTriggerSupplier;
//    private boolean m_isFieldCentric;
//    public static double distanceSetPoint = 0.0;
//
//    public static PIDCoefficients m_pidCoefficients = new PIDCoefficients(p, i, d);
//    private PIDFController m_pidController;
//
//    public AlignmentCommand(AlignmentSubsystem alignmentSubsystem, DriveSubsystem driveSubsystem, DoubleSupplier leftYSupplier,
//                            DoubleSupplier leftXSupplier, DoubleSupplier rightXSupplier, DoubleSupplier rightTriggerSupplier, boolean isFieldCentric) {
//        m_alignmentSubsystem = alignmentSubsystem;
//        m_driveSubsystem = driveSubsystem;
//        m_leftXSupplier = leftXSupplier;
//        m_leftYSupplier = leftYSupplier;
//        m_rightXSupplier = rightXSupplier;
//        m_rightTriggerSupplier = rightTriggerSupplier;
//        m_isFieldCentric = isFieldCentric;
//
//        addRequirements(m_alignmentSubsystem);
//        addRequirements(m_driveSubsystem);
//
//
//    }
//
//    @Override
//    public void initialize()
//    {
//        m_pidController = new PIDFController(m_pidCoefficients);
//        m_pidController.setTargetPosition(distanceSetPoint);
//    }
//    @Override
//    public void execute()
//    {
//        if(m_alignmentSubsystem.getBeamBreakState()){
////          if(false){
//            m_driveSubsystem.drive(m_leftXSupplier.getAsDouble(),
//                    m_leftYSupplier.getAsDouble(),
//                    m_rightXSupplier.getAsDouble(),
//                    m_rightTriggerSupplier.getAsDouble(),
//                    m_isFieldCentric);
//
//        }
//        else
//        {
//            double lateralDrive = m_pidController.update(m_alignmentSubsystem.getDistanceDifference());
//
//            m_driveSubsystem.drive(lateralDrive,
//                    0.0,
//                    0.0,
//                    0.0,
//                    false);
//        }
//        m_driveSubsystem.update();
//    }
//}
