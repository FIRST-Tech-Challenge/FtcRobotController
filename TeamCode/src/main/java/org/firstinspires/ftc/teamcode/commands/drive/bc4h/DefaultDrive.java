package org.firstinspires.ftc.teamcode.commands.drive.bc4h;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.bc4h.BC4HDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final BC4HDriveSubsystem m_drive;

    private final DoubleSupplier m_ls_y;
    private final DoubleSupplier m_ls_x;
    private final DoubleSupplier m_rs_x;

    private Telemetry telemetry;


    public DefaultDrive(BC4HDriveSubsystem subsystem, DoubleSupplier ls_y, DoubleSupplier ls_x, DoubleSupplier rs_x){
        m_drive = subsystem;
        m_ls_y = ls_y;
        m_ls_x = ls_x;
        m_rs_x = rs_x;
        addRequirements(subsystem);
    }

    public DefaultDrive(BC4HDriveSubsystem subsystem, DoubleSupplier ls_y, DoubleSupplier ls_x, DoubleSupplier rs_x, Telemetry telemetry){
        m_drive = subsystem;
        m_ls_y = ls_y;
        m_ls_x = ls_x;
        m_rs_x = rs_x;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void execute(){

        double y = m_ls_y.getAsDouble();
        double x = -m_ls_x.getAsDouble() * 1.1; // * 1.1 Counteract imperfect strafing
        double rx = -m_rs_x.getAsDouble();
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.BACK_LEFT, backLeftPower);
        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.BACK_RIGHT, backRightPower);
        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.FRONT_LEFT, frontLeftPower);
        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.FRONT_RIGHT, frontRightPower);

        

    }

}
