package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.armPIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.InterpLUT;
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
        pivotLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pivotRight = new MotorEx(map,"pivotRight");//tbd
        pivotRight.setInverted(false);
        pivotRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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
        dashboardTelemetry.addData("COMAngle", aCOMAngle());
        dashboardTelemetry.addData("balanceAngle", aBalanceAngle());
        dashboardTelemetry.addData("FF", calculateFeedForward());
        dashboardTelemetry.addData("rightEncoder", rightEncoder.getPosition());
        dashboardTelemetry.addData("rightEncoder rev", rightEncoder.getRevolutions());
        dashboardTelemetry.addData("leftEncoder rev", leftEncoder.getRevolutions());
        dashboardTelemetry.update();

    }

    public double setAngle(double setpoint){
        return m_pivotPID.calculate(setpoint);
    }


//    public void calculateFeedForward(double angle){
//         pivotFF = armMass*g*currentArmCOM*Math.sin(angle);
//    }
    public double calculateFeedForward(){
        return -kG * Math.cos(Math.toRadians(aCOMAngle()));
    }

    private double aCOMAngle(){
        return currentArmAngle+90 - aBalanceAngle();
    }

    private double aBalanceAngle(){
        return 3.59*currentArmLength + 119;
    }
    private double akG(double armLength) {
        return -0.0486*armLength + 0.228;
    }




    private void calcArmAngle() {
        currentArmAngle = 10.321*leftEncoder.getRevolutions();//linear equation (encoder to angle)
    }

    public void setMotors(double power){
        if(power>=0)
        {
            pivotRight.set(power+kS);
            pivotLeft.set(power+kS);
        }
        else{
            pivotRight.set(power-kS);
            pivotLeft.set(power-kS);

        }
    }
    public InstantCommand set(){
        return new InstantCommand(()->setMotors(calculateFeedForward()),this);
    }
    public InstantCommand stop(){
        return new InstantCommand(()->setMotors(0));
    }




}
