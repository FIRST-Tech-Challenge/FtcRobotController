package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.extensionConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BT.BTCommand;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.telescopicArmCOM;

public class ExtensionSubsystem implements Subsystem {
    private static ExtensionSubsystem instance;
    HardwareMap map;
    public MotorEx arm1;
    public MotorEx arm2;
    public Motor.Encoder extensionEncoder1;
    public Motor.Encoder extensionEncoder2;
    PIDController m_extensionPID;
    public double currentArmLength;
    public double armCOM;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
   
    public ExtensionSubsystem(HardwareMap map){
        this.map = map;
        arm1 = new MotorEx(map,"leftExtension");//hibur tbd
        arm2 =  new MotorEx(map,"rightExtension");//hibur tbd
        currentArmLength = encoderToLength(extensionEncoder1.getPosition());
        armCOM = telescopicArmCOM.calculateCenterOfMass(segmentMasses,segmentLengths,currentArmLength);
        m_extensionPID = new PIDController(eKP,eKI,eKD);


    }

    @Override
    public void periodic() {
        updateTelemetry();
        updateValues();
    }

    private ExtensionSubsystem()    {    }

    public double getArmLength()
    {
        return currentArmLength;
    }
//    public BTCommand setArmLength(double setpoint)
//    {
//        return new InstantCommand(m_extensionPID.calculate(setpoint));
//    }
    public double getArmCOM(){return armCOM;}

    private void updateValues() {
        armCOM = telescopicArmCOM.calculateCenterOfMass(segmentMasses,segmentLengths,currentArmLength);
    }

    private void updateTelemetry() {
        dashboard.addData("encoder1Value", extensionEncoder1.getPosition());
        dashboard.addData("encoder2Value", extensionEncoder2.getPosition());
        dashboard.addData("armLength ",currentArmLength);
    }
}
