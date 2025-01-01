package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.PID.PIDFController;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase {
    public DoubleSupplier armLength;
    public DoubleSupplier armCOM;
    public double currentArmCOM;
    public double currentArmLength;
    public double currentArmAngle;
    HardwareMap map;
    public MotorEx pivotLeft;
    public MotorEx pivotRight;
    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public
    PIDController m_pivotPID;
    private double pivotFF;


    public PivotSubsystem(HardwareMap map, DoubleSupplier armLength, DoubleSupplier armCOM){
        this.map = map;
        pivotLeft = new MotorEx(map,"pivotLeft");//tbd
        pivotRight = new MotorEx(map,"pivotRight");//tbd
        this.armLength = armLength;
        this.armCOM = armCOM;
        currentArmCOM = armCOM.getAsDouble();
        m_pivotPID = new PIDController(pKP,pKI,pKD);
        pivotFF = armMass*g*currentArmCOM*Math.sin(currentArmAngle);
        leftEncoder = pivotLeft.encoder;
        rightEncoder = pivotRight.encoder;
    }

    @Override
    public void periodic(){
        updateValues();
        updateTelemetry();
    }


    
    private void updateValues() {
        currentArmLength = armLength.getAsDouble();
        currentArmCOM = armCOM.getAsDouble();
        calcArmAngle();
    }



    private void updateTelemetry() {
        dashboardTelemetry.addData("armAngle", currentArmAngle);
        dashboardTelemetry.addData("rightEncoder", rightEncoder.getPosition());
        dashboardTelemetry.addData("rightEncoder rev", rightEncoder.getRevolutions());
        dashboardTelemetry.update();
    }

    public double setAngle(double setpoint){
        return m_pivotPID.calculate(setpoint);
    }

    private double setPivotMotor(double setpoint) {
        double pidResult = m_pivotPID.calculate(setpoint);
        return pidResult + pivotFF;
    }

//    public void calculateFeedForward(double angle){
//         pivotFF = armMass*g*currentArmCOM*Math.sin(angle);
//    }
    public void calculateFeedForward(double angle){
         pivotFF = akG(armLength)*Math.cos(aMaxAngle(armLength)-currentArmAngle);
    }

    private double akG(DoubleSupplier armLength) {
        return 0;
    }

    private double aMaxAngle(DoubleSupplier armLength) {
        return 0;
    }

    private void calcArmAngle() {
        currentArmAngle =  -10.4106*rightEncoder.getRevolutions()+110.925;//linear equation (encoder to angle)
    }


}
