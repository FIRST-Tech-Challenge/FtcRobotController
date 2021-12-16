package org.firstinspires.ftc.teamcode.commands.drive.bc4h;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.cv.OpenCvShippingElementDetector;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.bc4h.BC4HDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.webcam.WebCamSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final BC4HDriveSubsystem m_drive;

    private final DoubleSupplier m_ls_y;
    private final DoubleSupplier m_ls_x;
    private final DoubleSupplier m_rs_x;
    private final BooleanSupplier bPressed;

    //set the controller stick values
    private double controller1lstickx = 0.0;
    private double controller1lsticky = 0.0;
    private double controller1rstickx = 0.0;

    private double slowdown = 1.0;
    private boolean bpressed = false;
    private boolean slowdownflag = false;

    private Telemetry telemetry;


    public DefaultDrive(BC4HDriveSubsystem subsystem, DoubleSupplier ls_y, DoubleSupplier ls_x, DoubleSupplier rs_x, final BooleanSupplier bPressed){
        m_drive = subsystem;
        m_ls_y = ls_y;
        m_ls_x = ls_x;
        m_rs_x = rs_x;
        this.bPressed = bPressed;
        addRequirements(subsystem);
    }

    public DefaultDrive(BC4HDriveSubsystem subsystem, DoubleSupplier ls_y, DoubleSupplier ls_x, DoubleSupplier rs_x, final BooleanSupplier bPressed, Telemetry telemetry){
        m_drive = subsystem;
        m_ls_y = ls_y;
        m_ls_x = ls_x;
        m_rs_x = rs_x;
        this.bPressed = bPressed;
        this.telemetry = telemetry;

        addRequirements(subsystem);
    }


    @Override
    public void execute(){

        //double y = m_ls_y.getAsDouble();
        //double x = -m_ls_x.getAsDouble() * 1.1; // * 1.1 Counteract imperfect strafing
        //double rx = -m_rs_x.getAsDouble();

        controller1lstickx = m_ls_x.getAsDouble() ;
        controller1lsticky = m_ls_y.getAsDouble();;
        controller1rstickx = m_rs_x.getAsDouble();
        if(controller1lstickx < 0.3 && controller1lstickx > -0.3)
            controller1lstickx = 0.0;
        if(controller1lsticky < 0.3 && controller1lsticky > -0.3)
            controller1lsticky = 0.0;
        if(controller1rstickx < 0.3 && controller1rstickx > -0.3)
            controller1rstickx = 0.0;

        if(!bpressed){
            if(slowdownflag){
                slowdown = 1;
                slowdownflag = false;
            }
            else if(!slowdownflag){
                slowdown = 0.5;
                slowdownflag = true;
            }
        }

        ///Add slowdown
        controller1lstickx = controller1lstickx * slowdown;
        controller1lsticky = controller1lsticky * slowdown;
        controller1rstickx = controller1rstickx * slowdown;

        double y = controller1lsticky;
        double x = -controller1lstickx * 1.1; // * 1.1 Counteract imperfect strafing
        double rx = -controller1rstickx;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator)*slowdown;
        double backLeftPower = ((y - x + rx) / denominator)*slowdown;
        double frontRightPower = ((y - x - rx) / denominator)*slowdown;
        double backRightPower = ((y + x - rx) / denominator)*slowdown;

        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.BACK_LEFT, backLeftPower);
        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.BACK_RIGHT, backRightPower);
        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.FRONT_LEFT, frontLeftPower);
        m_drive.setMotorPower(BC4HDriveSubsystem.motorName.FRONT_RIGHT, frontRightPower);

        

    }

}
