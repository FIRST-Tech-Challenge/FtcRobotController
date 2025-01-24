package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.extensionPIDCosntants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.aConstraint;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.pGoalTolerance;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.pGoalVelocityTolerance;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.pKD;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.pKI;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.pKP;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.pMaxOutput;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.pTolerance;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.vConstraint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;

public class ExtensionSubsystem extends SubsystemBase {
    HardwareMap map;
    public MotorEx extension1;
    public MotorEx extension2;
    public Motor.Encoder extensionEncoder1;
    public Motor.Encoder extensionEncoder2;
//    public DigitalChannel limitSwitch;
    ProfiledPIDController m_extensionPID;
    public double currentArmLength = 0;
    public double armCOM;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
   
    public ExtensionSubsystem(HardwareMap map){
        this.map = map;
        extension1 = new MotorEx(map,"backExtension");//hibur tbd
        extension2 =  new MotorEx(map,"frontExtension");//hibur tbd
        extensionEncoder1 = extension1.encoder;
        extensionEncoder2 = extension2.encoder;
        currentArmLength = extensionEncoder2.getRevolutions();
        m_extensionPID = new ProfiledPIDController(eKP,eKI,eKD,new TrapezoidProfile.Constraints(eMaxV,eMaxA));
        extensionEncoder1.reset();

    }

    public double getArmLength()
    {
        return currentArmLength;
    }


    @Override
    public void periodic() {
        updateTelemetry();
        updateValues();
        if(m_extensionPID.atGoal()){
            setMotors(0);
        } else{setMotors(m_extensionPID.calculate(currentArmLength));}
    }

    private void updateValues() {
        currentArmLength = extensionEncoder2.getRevolutions();
        m_extensionPID.setPID(eKP,eKI,eKD);
        m_extensionPID.setConstraints(new TrapezoidProfile.Constraints(vConstraint,aConstraint));
        m_extensionPID.setTolerance(eTolerance);
        m_extensionPID.setGoalTolerance(eGoalTolerance, eGoalVelocityTolerance);
        m_extensionPID.m_controller.setAccumilatorResetTolerance(eGoalTolerance);//TODO:look at this
        m_extensionPID.setIzone(eIzone);
    }

    private void updateTelemetry() {
        dashboardTelemetry.addData("encoder2 rev", extensionEncoder2.getRevolutions());
        dashboardTelemetry.addData("armLength ",currentArmLength);
        dashboardTelemetry.addData("pid value", m_extensionPID.calculate(currentArmLength));
        dashboardTelemetry.addData("motorCurrent", extension1.motorEx.getCurrent(CurrentUnit.AMPS));
//        dashboardTelemetry.addData("isClosed", limitSwitch.getState());
        dashboardTelemetry.update();
    }

    public void setMotors(double power){
        if(currentArmLength>-4)
        {
            extension1.set(power);
            extension2.set(power);
        }
        else
        {
            extension1.set(0);
            extension2.set(0);
        }
    }


    public BTCommand setExtension(double setpoint){
        return new InstantCommand(()-> m_extensionPID.setGoal(setpoint)).;
    }
    public Command setNegative(){
        return new InstantCommand(()-> setMotors(-0.2));
    }
    public Command disablePID(){
        return new InstantCommand(()->m_extensionPID.disable());
    }
    public Command enablePID(){
        return new InstantCommand(()->m_extensionPID.enable(currentArmLength));
    }
}




