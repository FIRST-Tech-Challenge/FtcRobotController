package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;

import java.util.function.DoubleSupplier;

public class PivotSubsystem implements Subsystem {
    public DoubleSupplier armLength;
    public DoubleSupplier armCOM;
    public double currentArmCOM;
    public double currentArmLength;
    public double currentArmAngle;
    HardwareMap map;
    public MotorEx pivot1;
    public MotorEx pivot2;
    public Motor.Encoder pivotEncoder1;
    public Motor.Encoder pivotEncoder2;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    public
    PIDController m_pivotPID;
    private double pivotFF;


    public PivotSubsystem(HardwareMap map, DoubleSupplier armLength, DoubleSupplier armCOM, MotorEx pivot1, MotorEx pivot2){
        this.map = map;
        this.pivot1 = pivot1;
        this.pivot2 = pivot2;
        this.armLength = armLength;
        this.armCOM = armCOM;
        currentArmCOM = armCOM.getAsDouble();
        m_pivotPID = new PIDController(pKP,pKI,pKD);
        pivotFF = armMass*g*currentArmCOM*Math.sin(currentArmAngle);



    }

    @Override
    public void periodic(){
    updateTelemetry();
    updateValues();
    }


    
    private void updateValues() {
        currentArmLength = armLength.getAsDouble();
        currentArmCOM = armCOM.getAsDouble();
        calcArmAngle();
        calculateFeedForward();
    }



    private void updateTelemetry() {
        dashboard.addData("armAngle", currentArmAngle);
        dashboard.addData("pivotEncoder1", pivotEncoder1.getPosition());
        dashboard.addData("pivotEncoder2", pivotEncoder2.getPosition());
    }

    public void setAngle(double setpoint){
        pivot1.set(setArmMotor(setpoint));
    }

    private double setPivotMotor(double setpoint) {

    }

    public void calculateFeedForward(){

        pivotFF = armMass*g*currentArmCOM*Math.sin(currentArmAngle);
    }

    private void calcArmAngle() {
        currentArmAngle = 0;//todo: calc formula
    }
}
