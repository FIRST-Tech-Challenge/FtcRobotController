package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class PlaneLauncherSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_launcherServo;
    double m_position;


    public static double minScale = 0.3;
    public static double maxScale = 0.97;


    public PlaneLauncherSubsystem(HardwareMap hardwareMap, Telemetry telemetry, double initial_position) {
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
      //  m_launcherServo = hardwareMap.get(Servo.class, "launcher");
        m_position = initial_position;
    }


    public void shoot() {
        m_position = 0.5;
    }

    public void reload() {
        m_position = 0.0;
    }

    public void actuate()
    {
        m_position = (m_position + 1.0) % 2.0;
    }
    public double getCassetPosition(){return m_position;}
    @Override
    public void periodic() {
       // m_launcherServo.setPosition(m_position);
       // m_launcherServo.scaleRange(minScale, maxScale);
//
       // m_telemetry.addData("launcher Pos: ", m_launcherServo.getPosition());
        m_telemetry.addData("Set Position: ", m_position);
        m_telemetry.update();
    }
}

