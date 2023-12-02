//package org.firstinspires.ftc.teamcode.commands;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
//
//import java.util.function.DoubleSupplier;
//
//public class AutoTargetingDriveCommand extends CommandBase {
//    private final DriveSubsystem m_driveSubsystem;
//    private final VisionSubsystem m_visionSubsystem;
//    private final DoubleSupplier m_leftYSupplier;
//    private final DoubleSupplier m_leftXSupplier;
//    private final DoubleSupplier m_rightXSupplier;
//    private boolean m_isFieldCentric;
//    public static PIDCoefficients m_pidCoefficients = new PIDCoefficients(2.0, 0.0, 0.1);
//    private PIDFController m_pidController;
//
//    public AutoTargetingDriveCommand(DriveSubsystem drive, VisionSubsystem vision, DoubleSupplier leftYSupplier,
//                                     DoubleSupplier leftXSupplier, DoubleSupplier  rightXSupplier, boolean isFieldCentric) {
//
//        m_driveSubsystem = drive;
//        m_leftXSupplier = leftXSupplier;
//        m_leftYSupplier = leftYSupplier;
//        m_rightXSupplier = rightXSupplier;
//        m_isFieldCentric = isFieldCentric;
//        m_visionSubsystem = vision;
//
//        addRequirements(m_driveSubsystem);
//        addRequirements(m_visionSubsystem);
//
//
//    }
//
//
//    @Override
//    public void initialize() {
//        m_pidController = new PIDFController(m_pidCoefficients);
//        m_pidController.setInputBounds(-Math.PI, Math.PI);
//    }
//
//    @Override
//    public void execute() {
//        Pose2d currentPose = m_driveSubsystem.getPoseEstimate();
//
//        double targetAngle = 0.0;
//        m_pidController.setTargetPosition(targetAngle);
////        double headingControl = m_pidController.update(m_visionSubsystem.getAngle()*Math.PI/180.0);
//
////        m_driveSubsystem.drive(m_leftXSupplier.getAsDouble(),
////        m_leftYSupplier.getAsDouble(),
////            headingControl,
////            m_isFieldCentric);
////        m_driveSubsystem.update();
//
//    }
//
//
//}
