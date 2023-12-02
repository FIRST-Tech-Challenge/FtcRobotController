package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class CassetSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_cassetServo;
    double m_position;

    public static double minScale = 0.3;
    public static double maxScale = 0.97;


    public CassetSubsystem(HardwareMap hardwareMap, Telemetry telemetry, double initial_position) {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
     //   m_cassetServo = hardwareMap.get(Servo.class, "casset");
        m_position = initial_position;


    }


    public void depositPosition() {
        m_position = minScale;
    }

    public void intakePosition() {
        m_position = maxScale;
    }

    public Command depositCasset(){
        return new InstantCommand(()-> this.depositPosition());
    }

    public void actuate()
    {
        m_position = (m_position + 1.0) % 2.0;
    }
    public double getCassetPosition(){return m_position;}
    @Override
    public void periodic() {
//        m_cassetServo.setPosition(m_position);
//        m_cassetServo.scaleRange(minScale, maxScale);
//
//        m_telemetry.addData("Servo Pos: ", m_cassetServo.getPosition());
//        m_telemetry.addData("Set Position: ", m_position);
//        m_telemetry.update();
    }
}
