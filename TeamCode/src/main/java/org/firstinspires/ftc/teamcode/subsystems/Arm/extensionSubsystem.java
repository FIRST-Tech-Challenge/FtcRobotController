package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.armConstants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.armConstants.extensionConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.telescopicArmCOM;

import java.util.function.DoubleSupplier;

public class extensionSubsystem implements Subsystem {
    private static extensionSubsystem instance;
    HardwareMap map;
    public MotorEx arm1;
    public MotorEx arm2;
    public Motor.Encoder extensionEncoder1;
    public Motor.Encoder extensionEncoder2;
    PIDController m_extensionPID;
    public double currentArmLength;
    public double armCOM;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
   
    public extensionSubsystem(HardwareMap map, MotorEx arm1, MotorEx arm2){
        this.map = map;
        this.arm1 = arm1;
        this.arm2 = arm2;
        currentArmLength = encoderToLength*extensionEncoder1.getPosition();
        armCOM = telescopicArmCOM.calculateCenterOfMass(segmentMasses,segmentLengths,currentArmLength);
        m_extensionPID = new PIDController(eKP,eKI,eKD);

    }

    @Override
    public void periodic() {
        updateTelemetry();
        updateValues();
    }

    private extensionSubsystem()    {    }

    public double getArmLength()
    {
        return currentArmLength;
    }

    private void updateValues() {
        armCOM = telescopicArmCOM.calculateCenterOfMass(segmentMasses,segmentLengths,armLength);
    }

    private void updateTelemetry() {
        dashboard.addData("armLength ",armLength);
    }
}
