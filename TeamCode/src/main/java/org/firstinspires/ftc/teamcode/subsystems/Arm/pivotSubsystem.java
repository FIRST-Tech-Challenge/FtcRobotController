package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.armConstants.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.extensionSubsystem.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;

import java.util.function.DoubleSupplier;

public class pivotSubsystem implements Subsystem {
    HardwareMap map;
    public MotorEx pivot1;
    public MotorEx pivot2;
    public Motor.Encoder pivotEncoder1;
    public Motor.Encoder pivotEncoder2;
    private Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();
    private DoubleSupplier getArmLength;
    private double currentArmLength;

    PIDController m_pivotPID;


    public pivotSubsystem(HardwareMap map, MotorEx pivot1, MotorEx pivot2){
        this.map = map;
        this.pivot1 = pivot1;
        this.pivot2 = pivot2;
        m_pivotPID = new PIDController(pKP,pKI,pKD);
        this.getArmLength = extensionSubsystem.getInstance().armLengthSupplier;


    }

    @Override
    public void periodic(){
    updateTelemetry();
    updateValues();
    }


    
    private void updateValues() {
        currentArmLength = getArmLength.getAsDouble();
    }

    private void updateTelemetry() {

    }

    public static double calculateFeedForward(){

        return 0;
    }
}
