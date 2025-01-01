package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.armPIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;

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
        pivotLeft.setInverted(true);
        pivotRight = new MotorEx(map,"pivotRight");//tbd
        pivotRight.setInverted(false);
        this.armLength = armLength;
        this.armCOM = armCOM;
        currentArmCOM = armCOM.getAsDouble();
        m_pivotPID = new PIDController(pKP,pKI,pKD);
        pivotFF = armMass*g*currentArmCOM*Math.sin(currentArmAngle);
        leftEncoder = pivotLeft.encoder;
        rightEncoder = pivotRight.encoder;
        leftEncoder.reset();
        rightEncoder.reset();
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
        dashboardTelemetry.addData("leftEncoder rev", leftEncoder.getRevolutions());
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
        return 0.26*(armLength.getAsDouble()/totalLength)+0.29;
    }

    private double aMaxAngle(DoubleSupplier armLength) {
        return -13*(armLength.getAsDouble()/totalLength)+210;
    }

    private void calcArmAngle() {
        currentArmAngle = 10.321*leftEncoder.getRevolutions();//linear equation (encoder to angle)
    }

    public void setMotors(){
        pivotRight.set(kS);
        pivotLeft.set(kS);
    }
    public InstantCommand set(){
        return new InstantCommand(()->setMotors(),this);
    }



}
